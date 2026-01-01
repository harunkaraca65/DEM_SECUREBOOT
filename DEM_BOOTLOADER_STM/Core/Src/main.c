/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Secure Bootloader Main Program Body (STM32G030)
  * @project        : Demedukit Secure OTA
  * @author         : hrnkrc
  * @date           : December 2025
  * @description    : Implements a secure, dual-slot bootloader with:
  * - AES-256 Hook for decryption
  * - Anti-Rollback protection
  * - SSD1306 OLED Visual Feedback
  * - Fault-tolerant A/B slot architecture
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306_ll.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "aes.h"
#include "sha256.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/**
 * @brief Firmware Header structure found at the start of Slot A/B.
 * This structure is used for version control and integrity checks.
 */
typedef struct {
    uint32_t Magic;          /**< Magic number (0x53454355) to identify valid FW */
    uint32_t Version;        /**< Incrementing version for Anti-rollback */
    uint32_t PayloadSize;    /**< Total size of the binary in bytes */
    uint32_t CRC32;          /**< CRC32 of the decrypted payload */
    uint8_t  Signature[32];  /**< HMAC-SHA256 signature for authentication */
} BL_AppHeader_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* --- BOOTLOADER PROTOCOL CONSTANTS --- */
#define BL_PROTOCOL_INIT         0xABU
#define BL_PROTOCOL_ACK          0xC1U
#define BL_PROTOCOL_NACK         0x7FU

#define FW_HEADER_TOTAL_SIZE 64

/* --- BOOTLOADER COMMANDS --- */
#define BL_CMD_ERASE             0x50U
#define BL_CMD_WRITE             0x51U
#define BL_CMD_JUMP              0x52U
#define BL_CMD_VERIFY            0x53U

/* --- MEMORY LAYOUT CONFIGURATION (STM32G030 - 64KB) --- */
#define BL_FLASH_PAGE_SIZE       0x800U      /* 2KB Page Size */
#define BL_SLOT_A_START          0x08004000U /* Active App Partition (Address) */
#define BL_SLOT_B_START          0x0800A000U /* OTA Download Partition (Address) */
#define BL_HEADER_MAGIC          0x53454355U /* "SECU" ASCII in Hex */
#define BL_SLOT_SIZE             (24 * 1024) /* Slot sizes are now 24KB (24 * 1024) */
#define FW_HEADER_TOTAL_SIZE 	  64
/* --- FLASH PROTECTION KEYS --- */
#define FLASH_KEY1               0x45670123U
#define FLASH_KEY2               0xCDEF89ABU
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
/* --- HIGH LEVEL LOGIC --- */
void BL_ListenForUpdate(void);
void BL_ProcessSecureCommands(void);
void BL_JumpToApplication(uint32_t Address);
void BL_UI_UpdateStatus(const char *StatusMsg, uint8_t Progress);


/* --- FLASH DRIVERS --- */
void BL_Flash_Unlock(void);
void BL_Flash_Lock(void);
bool BL_Flash_EraseRange(uint32_t StartAddress, uint32_t Size);
uint8_t BL_Flash_Write(uint32_t StartAddress, uint8_t *Data, uint32_t Length);
bool BL_Flash_ErasePage(uint32_t PageAddress);

/* --- SECURITY UTILS --- */
bool BL_Security_VerifyVersion(uint32_t NewVersion);
bool BL_Security_DecryptPacket(uint8_t *Ciphertext, uint8_t *Plaintext, uint32_t Len);
bool BL_Firmware_Activate(void);
bool BL_Security_VerifySignature(void);
uint32_t BL_CalculateCRC32_Soft(uint8_t *pData, uint32_t Length);

/* --- UART HELPERS --- */
uint8_t BL_UART_ReceiveByte(void);
void BL_UART_SendByte(uint8_t data);
void BL_UART_ReceiveArray(uint8_t *buffer, uint16_t len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

	/* SysTick_IRQn interrupt configuration */
	NVIC_SetPriority(SysTick_IRQn, 3);

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
SSD1306_Init();
	  LL_USART_Enable(USART2);
	  BL_UI_UpdateStatus("SYSTEM BOOT", 0);

	  // --- STEP 1: LISTEN FOR ESP32 HANDSHAKE ---
	  // Checks for new version via ESP32. Download logic resides here.
	  BL_ListenForUpdate();

	  // --- STEP 2: POST-OTA VALIDATION & ACTIVATION ---
	  // Offset compatible with Python script
	  BL_AppHeader_t *newFW = (BL_AppHeader_t *)BL_SLOT_B_START;

	  // 1. Check if Slot B has a valid signature (SECU)
	  if (newFW->Magic == BL_HEADER_MAGIC)
	  {
	      // 2. Prepare Payload for CRC Verification
	      uint8_t *payloadAddr = (uint8_t *)(BL_SLOT_B_START + FW_HEADER_TOTAL_SIZE);
	      uint32_t payloadSize = newFW->PayloadSize; // Note: Ensure this matches the struct member name

	      // 3. Calculate Integrity Checksum
	      uint32_t calculatedCRC = BL_CalculateCRC32_Soft(payloadAddr, payloadSize);

	      // Verify Version and CRC
	      bool isVersionOK = BL_Security_VerifyVersion(newFW->Version);
	      bool isCRC_OK    = (calculatedCRC == newFW->CRC32);

	      if (isVersionOK && isCRC_OK)
	      {
	          BL_UI_UpdateStatus("VERIFY SUCCESS", 100);
	          LL_mDelay(1000);

	          // 4. Activate: Copy Slot B (New) to Slot A (Active)
	          if (BL_Firmware_Activate())
	          {
	              /* --- CRITICAL: INVALIDATE SLOT B TO BREAK THE LOOP --- */
	              // Erase Slot B signature to prevent re-copying after reset.
	              BL_UI_UpdateStatus("FINALIZING...", 100);

	              /* --- LL / REGISTER LEVEL FLASH ERASE (SLOT B INVALIDATION) --- */

	              /* 1. Unlock Flash (Unlock Sequence) */
	              if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) != 0U) {
	                  WRITE_REG(FLASH->KEYR, 0x45670123U); // Key 1
	                  WRITE_REG(FLASH->KEYR, 0xCDEF89ABU); // Key 2
	              }

	              /* 2. Wait Until Flash Is Ready */
	              while (READ_BIT(FLASH->SR, FLASH_SR_BSY1) != 0U);

	              /* 3. Activate Programming Mode (PG Bit) */
	              SET_BIT(FLASH->CR, FLASH_CR_PG);

	              /* 4. Corrupt Slot B by writing 0 (STM32G0 requires two consecutive 32-bit writes) */
	              *(__IO uint32_t*)(BL_SLOT_B_START)      = 0x00000000U; // First 32-bit
	              *(__IO uint32_t*)(BL_SLOT_B_START + 4U) = 0x00000000U; // Second 32-bit (Triggers HW op)

	              /* 5. Wait for Process to Finish */
	              while (READ_BIT(FLASH->SR, FLASH_SR_BSY1) != 0U);

	              /* 6. Cleanup and Locking */
	              CLEAR_BIT(FLASH->CR, FLASH_CR_PG); // Clear PG bit
	              SET_BIT(FLASH->CR, FLASH_CR_LOCK); // Lock Flash again
	              /* ----------------------------------------------------- */

	              BL_UI_UpdateStatus("UPDATE COMPLETE", 100);
	              LL_mDelay(1000);

	              NVIC_SystemReset(); // Reboot to run the new Slot A application
	          }
	      }
	      else
	      {
	          // Validation failed
	          BL_UI_UpdateStatus(isVersionOK ? "CRC MISMATCH!" : "VERSION OLD!", 0);
	          LL_mDelay(3000);
	      }
	  }

	  // --- STEP 3: LAUNCH APPLICATION (FALLBACK) ---
	  // If no update was found or update was just finished, jump to Slot A.
	  BL_UI_UpdateStatus("LAUNCHING APP", 100);
	  LL_mDelay(500);
	  BL_JumpToApplication(BL_SLOT_A_START + FW_HEADER_TOTAL_SIZE);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* Error Trap: If Jump fails, blink Red LED */
		    LL_GPIO_TogglePin(GPIOD, LL_GPIO_PIN_3);
		    LL_mDelay(200);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
	while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2) {
	}

	/* HSI configuration and activation */
	LL_RCC_HSI_Enable();
	while (LL_RCC_HSI_IsReady() != 1) {
	}

	/* Main PLL configuration and activation */
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 8,
	LL_RCC_PLLR_DIV_2);
	LL_RCC_PLL_Enable();
	LL_RCC_PLL_EnableDomain_SYS();
	while (LL_RCC_PLL_IsReady() != 1) {
	}

	/* Set AHB prescaler*/
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

	/* Sysclk activation on the main PLL */
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
	}

	/* Set APB1 prescaler*/
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

	LL_Init1msTick(64000000);

	/* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
	LL_SetSystemCoreClock(64000000);
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	LL_I2C_InitTypeDef I2C_InitStruct = { 0 };

	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_PCLK1);

	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
	/**I2C1 GPIO Configuration
	 PB6   ------> I2C1_SCL
	 PB7   ------> I2C1_SDA
	 */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */

	/** I2C Initialization
	 */
	I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
	I2C_InitStruct.Timing = 0x00C12166;
	I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
	I2C_InitStruct.DigitalFilter = 0;
	I2C_InitStruct.OwnAddress1 = 0;
	I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
	I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
	LL_I2C_Init(I2C1, &I2C_InitStruct);
	LL_I2C_EnableAutoEndMode(I2C1);
	LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);
	LL_I2C_DisableOwnAddress2(I2C1);
	LL_I2C_DisableGeneralCall(I2C1);
	LL_I2C_EnableClockStretching(I2C1);
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	LL_USART_InitTypeDef USART_InitStruct = { 0 };

	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);

	/* Peripheral clock enable */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
	/**USART1 GPIO Configuration
	 PA9   ------> USART1_TX
	 PA10   ------> USART1_RX
	 */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
	USART_InitStruct.BaudRate = 115200;
	USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
	USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
	USART_InitStruct.Parity = LL_USART_PARITY_NONE;
	USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
	USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
	LL_USART_Init(USART1, &USART_InitStruct);
	LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
	LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
	LL_USART_DisableFIFO(USART1);
	LL_USART_ConfigAsyncMode(USART1);

	/* USER CODE BEGIN WKUPType USART1 */

	/* USER CODE END WKUPType USART1 */

	LL_USART_Enable(USART1);

	/* Polling USART1 initialisation */
	while ((!(LL_USART_IsActiveFlag_TEACK(USART1)))
			|| (!(LL_USART_IsActiveFlag_REACK(USART1)))) {
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	LL_USART_InitTypeDef USART_InitStruct = { 0 };

	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
	/**USART2 GPIO Configuration
	 PA2   ------> USART2_TX
	 PA3   ------> USART2_RX
	 */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USART2 interrupt Init */
	NVIC_SetPriority(USART2_IRQn, 0);
	NVIC_EnableIRQ(USART2_IRQn);

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
	USART_InitStruct.BaudRate = 115200;
	USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
	USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
	USART_InitStruct.Parity = LL_USART_PARITY_NONE;
	USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
	USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
	LL_USART_Init(USART2, &USART_InitStruct);
	LL_USART_ConfigAsyncMode(USART2);

	/* USER CODE BEGIN WKUPType USART2 */

	/* USER CODE END WKUPType USART2 */

	LL_USART_Enable(USART2);

	/* Polling USART2 initialisation */
	while ((!(LL_USART_IsActiveFlag_TEACK(USART2)))
			|| (!(LL_USART_IsActiveFlag_REACK(USART2)))) {
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOF);
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOD);
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

	/**/
	LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_1);

	/**/
	LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_2);

	/**/
	LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_3);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
* ========================================================================== */
/* CORE LOGIC IMPLEMENTATION                                                  */
/* ========================================================================== */

/**
 * @brief  Listens for the magic byte from ESP32 for 3 seconds.
 * If received, enters Command Processing mode.
 */
void BL_ListenForUpdate(void) {
    uint32_t timeout_counter = 300; // 300 * 10ms = 3 Seconds

    BL_UI_UpdateStatus("WAITING OTA...", 0);

    while (timeout_counter > 0) {
        /* Visual Feedback: Blink LED */
        if (timeout_counter % 20 == 0) LL_GPIO_TogglePin(GPIOD, LL_GPIO_PIN_3);

        /* Check UART RX Buffer */
        if (LL_USART_IsActiveFlag_RXNE(USART2)) {
            if (LL_USART_ReceiveData8(USART2) == BL_PROTOCOL_INIT) {
                /* Handshake Successful */
                LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_3); // Solid Red LED
                BL_UART_SendByte(BL_PROTOCOL_ACK);          // Send ACK
                BL_ProcessSecureCommands();                 // Enter Secure Loop
                return;
            }
        }
        LL_mDelay(10);
        timeout_counter--;
    }

    /* Timeout: Clean up UI */
    BL_UI_UpdateStatus("STARTING APP", 100);
    LL_mDelay(500); // Small delay to read screen
}

/**
 * @brief  Main loop to process secure commands (Erase, Write, Verify).
 * Handles Slot B management and Decryption hooks.
 */
void BL_ProcessSecureCommands(void) {
    uint8_t rxBuffer[128];
    uint32_t currentWriteAddr = BL_SLOT_B_START;
    uint32_t bytesReceived = 0;

    BL_Flash_Unlock();

    while (1) {
        uint8_t command = BL_UART_ReceiveByte();

        switch (command) {
            /* --- ERASE COMMAND (PREPARATION ONLY) --- */
            case BL_CMD_ERASE:
                // Notify user but do not erase (Smart erase happens during write)
                BL_UI_UpdateStatus("READY...", 0);

                // Reset addresses
                currentWriteAddr = BL_SLOT_B_START;
                bytesReceived = 0;

                // Send immediate ACK to ESP32, do not stall
                BL_UART_SendByte(BL_PROTOCOL_ACK);
                break;

            /* --- WRITE COMMAND (SMART ERASE HERE) --- */
            case BL_CMD_WRITE:
                {
                    uint8_t packetLen = BL_UART_ReceiveByte();
                    BL_UART_ReceiveArray(rxBuffer, packetLen);

                    /* --- CRITICAL POINT: Page Start Check --- */
                    /* If current address is page-aligned (multiple of 2048), erase that page first */
                    if ((currentWriteAddr % BL_FLASH_PAGE_SIZE) == 0) {
                        // Attempt recovery in case of error
                        if (!BL_Flash_ErasePage(currentWriteAddr)) {
                             BL_UART_SendByte(BL_PROTOCOL_NACK);
                             break;
                        }
                    }

                    /* Decrypt (Enable here if needed in future) */
                    // BL_Security_DecryptPacket(rxBuffer, rxBuffer, packetLen);

                    /* Write Data */
                    if (BL_Flash_Write(currentWriteAddr, rxBuffer, packetLen) == 1) {
                        currentWriteAddr += packetLen;
                        bytesReceived += packetLen;

                        /* UI Update (Every 1KB) */
                        if (bytesReceived % 1024 == 0) {
                             // Percentage calc: (Received / Total Slot Size) * 100
                             uint8_t progress = (bytesReceived * 100) / BL_SLOT_SIZE;
                             BL_UI_UpdateStatus("DOWNLOADING...", progress);
                        }

                        BL_UART_SendByte(BL_PROTOCOL_ACK);
                    } else {
                        BL_UART_SendByte(BL_PROTOCOL_NACK);
                    }
                }
                break;

            /* --- VERIFY COMMAND --- */
            case BL_CMD_VERIFY:
                {
                    BL_UI_UpdateStatus("VERIFYING...", 95);
                    BL_AppHeader_t *newHeader = (BL_AppHeader_t *)BL_SLOT_B_START;

                    if (newHeader->Magic == BL_HEADER_MAGIC) {
                        if (BL_Security_VerifyVersion(newHeader->Version)) {
                             // Signature check can be added: && BL_Security_VerifySignature()
                            BL_UI_UpdateStatus("INTEGRITY OK", 100);
                            BL_UART_SendByte(BL_PROTOCOL_ACK);
                        } else {
                            BL_UI_UpdateStatus("ROLLBACK BLOCKED", 0);
                            BL_UART_SendByte(BL_PROTOCOL_NACK);
                        }
                    } else {
                        BL_UI_UpdateStatus("INVALID HEADER", 0);
                        BL_UART_SendByte(BL_PROTOCOL_NACK);
                    }
                }
                break;

            /* --- JUMP COMMAND --- */
            case BL_CMD_JUMP:
                BL_UI_UpdateStatus("REBOOTING...", 100);
                BL_Flash_Lock();
                BL_UART_SendByte(BL_PROTOCOL_ACK);
                LL_mDelay(100);
                NVIC_SystemReset();
                return;

            default:
                BL_UART_SendByte(BL_PROTOCOL_NACK);
                break;
        }
    }
}



/* ========================================================================== */
/* UI & HELPER FUNCTIONS                                                      */
/* ========================================================================== */

/**
 * @brief  Helper to update OLED status and progress bar.
 */
void BL_UI_UpdateStatus(const char *StatusMsg, uint8_t Progress) {
    SSD1306_Clear();
    /* Draw Header */
    SSD1306_WriteString(30, 2, "DEMEDUKIT");
    /* Draw Status Message */
    SSD1306_WriteString(10, 15, (char*)StatusMsg);
    /* Draw Progress Bar */
    SSD1306_DrawProgressBar(Progress);
    SSD1306_UpdateScreen();
}

/**
 * @brief  Jumps to the User Application located at the specified address.
 */
typedef void (*pFunction)(void); // Function pointer definition

void BL_JumpToApplication(uint32_t address)
{
    // 1. Is there valid code at destination? (Is Stack Pointer in RAM?)
    // STM32G0 RAM usually starts at 0x20000000
    uint32_t msp = *(__IO uint32_t*)address;

    if ((msp & 0x2FF00000) != 0x20000000) {
        BL_UI_UpdateStatus("EMPTY APP!", 0); // Error if address is empty
        return;
    }

    // 2. DISABLE ALL INTERRUPTS (Global Interrupt Disable)
    __disable_irq();

    // 3. Kill SysTick Timer (Main culprit for jump issues!)
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    // 4. Disable Peripherals (UART, etc.)
    // It is good practice to de-init any other LL inits.
    // Example: Disabling USART2
    if (USART2->CR1 & USART_CR1_UE) {
        CLEAR_BIT(USART2->CR1, USART_CR1_UE);
    }

    // 5. NVIC (Interrupt Controller) Cleanup
    // Clearing all pending interrupt flags
    for (int i = 0; i < 32; i++) {
        NVIC->ICER[0] = (1 << i); // Disable interrupt
        NVIC->ICPR[0] = (1 << i); // Clear pending flag
    }

    // 6. Jump Preparation
    uint32_t jump_addr = *(__IO uint32_t*)(address + 4); // Reset Handler address
    pFunction jump_to_app = (pFunction)jump_addr;

    // 7. Set Main Stack Pointer (MSP) to Application's
    __set_MSP(msp);


    jump_to_app();
}

/* ========================================================================== */
/* FLASH DRIVERS (LL IMPLEMENTATION)                                          */
/* ========================================================================== */

void BL_Flash_Unlock(void) {
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
    }
}

void BL_Flash_Lock(void) {
    FLASH->CR |= FLASH_CR_LOCK;
}

bool BL_Flash_EraseRange(uint32_t StartAddress, uint32_t Size) {
    uint32_t startPage = (StartAddress - 0x08000000U) / BL_FLASH_PAGE_SIZE;
    uint32_t nbPages = (Size + BL_FLASH_PAGE_SIZE - 1) / BL_FLASH_PAGE_SIZE;
    uint32_t timeout;

    /* 1. Clear Possible Old Error Flags (Critical!) */
    FLASH->SR = FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR |
                FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR |
                FLASH_SR_MISERR | FLASH_SR_FASTERR | FLASH_SR_EOP;

    for (uint32_t i = 0; i < nbPages; i++) {

        /* 2. Disable Interrupts Only For Critical Moment */
        __disable_irq();

        /* Wait for previous operation to complete (Timeout protected) */
        timeout = 0xFFFF;
        while ((FLASH->SR & FLASH_SR_BSY1) && timeout > 0) timeout--;

        if (timeout == 0) { __enable_irq(); return false; } // Flash not responding

        /* Prepare Erase Commands */
        FLASH->CR |= FLASH_CR_PER;
        FLASH->CR &= ~FLASH_CR_PNB;
        FLASH->CR |= ((startPage + i) << FLASH_CR_PNB_Pos);
        FLASH->CR |= FLASH_CR_STRT;

        /* Wait for Process to Finish */
        timeout = 0xFFFFFF; // Increasing timeout
        while ((FLASH->SR & FLASH_SR_BSY1) && timeout > 0) timeout--;

        /* Error Check */
        if ((timeout == 0) || (FLASH->SR & (FLASH_SR_WRPERR | FLASH_SR_PGAERR))) {
            FLASH->CR &= ~FLASH_CR_PER;
            __enable_irq();
            return false; // Erase failed
        }

        /* 3. Re-enable Interrupts (Let system breathe) */
        __enable_irq();

        /* Optional: Add here if using Watchdog:
           LL_IWDG_ReloadCounter(IWDG);
        */
    }

    /* Cleanup */
    FLASH->CR &= ~FLASH_CR_PER;
    return true;
}

/**
 * @brief  Optimized function that erases a single page.
 * @param  PageAddress: Start address of the page to be erased.
 * @return True if successful, false otherwise.
 */
bool BL_Flash_ErasePage(uint32_t PageAddress) {
    // Check if address is in valid flash range (Optional but recommended)
    if (PageAddress < BL_SLOT_B_START || PageAddress >= (BL_SLOT_B_START + BL_SLOT_SIZE)) {
        return false; // Do not go out of Slot B!
    }

    uint32_t pageIndex = (PageAddress - 0x08000000U) / BL_FLASH_PAGE_SIZE;

    /* 1. Clear Possible Error Flags */
    FLASH->SR = FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR |
                FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR |
                FLASH_SR_MISERR | FLASH_SR_FASTERR | FLASH_SR_EOP;

    /* 2. Disable Interrupts (Very briefly) */
    __disable_irq();

    /* Wait for previous process to finish */
    while (FLASH->SR & FLASH_SR_BSY1);

    /* Prepare Erase Commands */
    FLASH->CR |= FLASH_CR_PER;                // Page Erase Active
    FLASH->CR &= ~FLASH_CR_PNB;               // Clear PNB first
    FLASH->CR |= (pageIndex << FLASH_CR_PNB_Pos); // Which page?
    FLASH->CR |= FLASH_CR_STRT;               // Start!

    /* Wait (Takes only 20-40ms) */
    while (FLASH->SR & FLASH_SR_BSY1);

    /* Cleanup */
    FLASH->CR &= ~FLASH_CR_PER;

    /* Any error? */
    if (FLASH->SR & (FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PROGERR | FLASH_SR_OPERR)) {
        __enable_irq();
        return false;
    }

    /* 3. Enable Interrupts */
    __enable_irq();
    return true;
}

uint8_t BL_Flash_Write(uint32_t StartAddress, uint8_t *Data, uint32_t Length) {
    uint32_t dataWord1, dataWord2;

    __disable_irq();
    FLASH->SR = FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR |
                FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR |
                FLASH_SR_MISERR | FLASH_SR_FASTERR | FLASH_SR_EOP;

    FLASH->CR |= FLASH_CR_PG;

    for (uint32_t i = 0; i < Length; i += 8) {
        /* Prepare Double Word (64-bit) for STM32G0 */
        dataWord1 = *(uint32_t*)(&Data[i]);
        dataWord2 = *(uint32_t*)(&Data[i + 4]);

        *(__IO uint32_t*)(StartAddress + i) = dataWord1;
        *(__IO uint32_t*)(StartAddress + i + 4) = dataWord2;

        while (FLASH->SR & FLASH_SR_BSY1);

        if (FLASH->SR & (FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR)) {
            FLASH->CR &= ~FLASH_CR_PG;
            __enable_irq();
            return 0; // Write Error
        }
    }

    FLASH->CR &= ~FLASH_CR_PG;
    __enable_irq();
    return 1; // Success
}

/* ========================================================================== */
/* SECURITY & UART UTILITIES                                                  */
/* ========================================================================== */

bool BL_Security_VerifyVersion(uint32_t NewVersion) {
    BL_AppHeader_t *currentHeader = (BL_AppHeader_t *)BL_SLOT_A_START;

    /* If Slot A is empty/invalid, accept any version */
    if (currentHeader->Magic != BL_HEADER_MAGIC) return true;

    /* Reject if NewVersion <= CurrentVersion (Anti-Rollback) */
    return (NewVersion > currentHeader->Version);
}

/* Updated UART Functions with professional naming */
uint8_t BL_UART_ReceiveByte(void) {
    while (!LL_USART_IsActiveFlag_RXNE(USART2));
    return LL_USART_ReceiveData8(USART2);
}

void BL_UART_ReceiveArray(uint8_t *buffer, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        buffer[i] = BL_UART_ReceiveByte();
    }
}

void BL_UART_SendByte(uint8_t data) {
    LL_USART_TransmitData8(USART2, data);
    while (!LL_USART_IsActiveFlag_TC(USART2));
}
/**
 * @brief  Activates firmware by copying it from Slot B to Slot A.
 * @return true if successful, false otherwise.
 */
bool BL_Firmware_Activate(void)
{
    BL_AppHeader_t *pHeader = (BL_AppHeader_t *)BL_SLOT_B_START;
    uint32_t payloadSize = pHeader->PayloadSize;

    /* Security: Do not process if size is too large or 0 */
    if (payloadSize == 0 || payloadSize > (32 * 1024)) return false;

    // Total size to copy (Header + Payload)
    // Since STM32G0 writes 64-bit, size should be a multiple of 8.
    uint32_t totalSize = payloadSize + 64; // FW_HEADER_SIZE

    /* ---------------------------------------------------------------------- */
    /* 1. UNLOCK FLASH                                                        */
    /* ---------------------------------------------------------------------- */
    if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) != 0U) {
        WRITE_REG(FLASH->KEYR, 0x45670123U);
        WRITE_REG(FLASH->KEYR, 0xCDEF89ABU);
    }
    // Wait
    while (READ_BIT(FLASH->SR, FLASH_SR_BSY1) != 0U);

    /* ---------------------------------------------------------------------- */
    /* 2. ERASE SLOT A                                                        */
    /* ---------------------------------------------------------------------- */
    // Erasing page by page based on Slot A size (2KB per page)
    uint32_t pagesToErase = (totalSize / 2048) + 1;
    uint32_t pageAddr = BL_SLOT_A_START;

    for (uint32_t i = 0; i < pagesToErase; i++)
    {
        // Calculate Page Number (Addr - Base) / 2048
        uint32_t pageNum = (pageAddr - 0x08000000) / 2048;

        // a. Erase Settings
        MODIFY_REG(FLASH->CR, FLASH_CR_PNB, (pageNum << FLASH_CR_PNB_Pos)); // Page Num
        SET_BIT(FLASH->CR, FLASH_CR_PER);  // Page Erase Mode

        // b. Start Erase
        SET_BIT(FLASH->CR, FLASH_CR_STRT);

        // c. Wait
        while (READ_BIT(FLASH->SR, FLASH_SR_BSY1) != 0U);

        // Error check
        if (FLASH->SR & (FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR)) {
             // Clear error flags
             FLASH->SR |= (FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR);
             CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
             SET_BIT(FLASH->CR, FLASH_CR_LOCK);
             return false;
        }

        pageAddr += 2048; // Next page
    }

    // Exit erase mode
    CLEAR_BIT(FLASH->CR, FLASH_CR_PER);

    /* ---------------------------------------------------------------------- */
    /* 3. COPY LOOP (SLOT B -> SLOT A)                                        */
    /* ---------------------------------------------------------------------- */
    // STM32G0 writes 64-bit (Double Word). Two 32-bit words must be written consecutively.

    uint32_t *pSrc = (uint32_t *)BL_SLOT_B_START;
    uint32_t *pDest = (uint32_t *)BL_SLOT_A_START;

    // Enable programming mode
    SET_BIT(FLASH->CR, FLASH_CR_PG);

    for (uint32_t i = 0; i < totalSize; i += 8)
    {
        // 1. Write first 32-bit word
        *pDest = *pSrc;
        pDest++; pSrc++;

        // 2. Write second 32-bit word (Write operation triggered here!)
        *pDest = *pSrc;
        pDest++; pSrc++;

        // 3. Wait for operation to complete
        while (READ_BIT(FLASH->SR, FLASH_SR_BSY1) != 0U);

        // 4. Write Error Check
        if (FLASH->SR & (FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR)) {
             FLASH->SR |= (FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR);
             CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
             SET_BIT(FLASH->CR, FLASH_CR_LOCK);
             return false;
        }
    }

    /* ---------------------------------------------------------------------- */
    /* 4. CLOSING (LOCK)                                                      */
    /* ---------------------------------------------------------------------- */
    CLEAR_BIT(FLASH->CR, FLASH_CR_PG); // Disable PG bit
    SET_BIT(FLASH->CR, FLASH_CR_LOCK); // Lock Flash

    return true;
}

/**
 * @brief  AES-256 CBC Decryption Implementation.
 * @param  Ciphertext: Data received from ESP32.
 * @param  Plaintext: Decrypted output to be written to Flash.
 * @param  Len: Data length (must be multiple of 16).
 */
bool BL_Security_DecryptPacket(uint8_t *Ciphertext, uint8_t *Plaintext, uint32_t Len) {
    /* KEY and IV: Must match exactly with the Kali Linux Python script */
    static const uint8_t AES_KEY[32] = { 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF,
                                         0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF,
                                         0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF,
                                         0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF };

    static const uint8_t AES_IV[16]  = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                         0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };

    struct AES_ctx ctx;

    /* Initialize AES context with Key and Initialization Vector (IV) */
    AES_init_ctx_iv(&ctx, AES_KEY, AES_IV);

    /* Decrypt buffer in-place */
    AES_CBC_decrypt_buffer(&ctx, Ciphertext, Len);

    return true;
}

/**
 * @brief  Verifies the Digital Signature of the firmware in Slot B.
 * Uses SHA-256 to compare against the signature in BL_AppHeader_t.
 */
bool BL_Security_VerifySignature(void) {
    BL_AppHeader_t *header = (BL_AppHeader_t *)BL_SLOT_B_START;
    uint8_t calculated_hash[32];
    SHA256_CTX ctx; // Type will be recognized when library is included

    sha256_init(&ctx);
    // Calculates hash of the actual firmware payload (after Header)
    sha256_update(&ctx, (uint8_t*)(BL_SLOT_B_START + sizeof(BL_AppHeader_t)), header->PayloadSize);
    sha256_final(&ctx, calculated_hash);

    /* Seal check: Compare calculated hash with signature in header */
    if (memcmp(calculated_hash, header->Signature, 32) == 0) {
        return true;
    }
    return false;
}
uint32_t BL_CalculateCRC32_Soft(uint8_t *pData, uint32_t Length)
{
    uint32_t crc = 0xFFFFFFFF; // Initial Value

    for (uint32_t i = 0; i < Length; i++)
    {
        crc ^= pData[i];
        for (uint32_t j = 0; j < 8; j++)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320; // Standard Ethernet/Zip Polynomial
            else
                crc >>= 1;
        }
    }
    return ~crc; // Result is inverted (NOT operation)
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
