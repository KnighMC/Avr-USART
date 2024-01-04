# FUTURE_LIBRARY_UPDATES

A more advanced approach is being taken for this library. Where in addition to the classic configuration
functions for the USART module, it is intended to incorporate more advanced options for its application in embedded systems.

# AVAILABLE DEVICES

- ATMEGA328PB
- ATMEGA328P

# IGNORE

typedef struct {

    uint16_t  BaudRate;             // This field defines expected USART communication Baud Rate.
    uint8_t   DataWidth;            // Specifies the number of data bits to transmit or receive in a frame.
    uint8_t   StopBits;             // Specifies the number of stop bits to transmit.
    uint8_t   Parity;               // Specifies the parity mode.
    uint8_t   TransferMode;         // Specifies whether the Receive and/or Transmit mode is enabled or disabled.
    bool      SoftwareFlowControl;  // Specifies whether the Software flow control mode is enabled or disabled.
    bool      OverSampling;         // Specifies whether USART oversampling mode is 16 or 8.

} USART_RegisterSettings;

/**********************************************************************************
 * When the USART_RegisterSettings.SoftwareFlowControl is enable(1) the struct is
 * a Setup of Usart control. At compile time, it performs evaluations (asserts) on
 * all USART configurations. If during compilation it is detected that any
 * parameter does not correspond to a configuration available for the USART module,
 * the compilation will stop.
 **********************************************************************************/

#define assert_param(expr) ((void)0)                /* Debug in compile time  */

#define USART_MAX_BAUDRATE          12500000U       /* Max value of Baud Rate */
#define USART_DIRECTION_NONE        
#define USART_DIRECTION_RX
#define USART_DIRECTION_TX
#define USART_DIRECTION_TX_RX
#define USART_PARITY_NONE
#define USART_PARITY_EVEN
#define USART_PARITY_ODD
#define USART_DATAWIDTH_5B
#define USART_DATAWIDTH_6B
#define USART_DATAWIDTH_7B
#define USART_DATAWIDTH_8B
#define USART_DATAWIDTH_9B
#define USART_OVERSAMPLING_16
#define USART_OVERSAMPLING_8
#define USART_STOPBITS_1
#define USART_STOPBITS_2


#define IS_USART_BAUDRATE(__BAUDRATE__)     ((__BAUDRATE__) <= USART_MAX_BAUDRATE)

#define IS_USART_DIRECTION(__VALUE__)       (((__VALUE__) == USART_DIRECTION_NONE) \
                                          || ((__VALUE__) == USART_DIRECTION_RX) \
                                          || ((__VALUE__) == USART_DIRECTION_TX) \
                                          || ((__VALUE__) == USART_DIRECTION_TX_RX))

#define IS_USART_PARITY(__VALUE__)          (((__VALUE__) == USART_PARITY_NONE) \
                                          || ((__VALUE__) == USART_PARITY_EVEN) \
                                          || ((__VALUE__) == USART_PARITY_ODD))

#define IS_USART_DATAWIDTH(__VALUE__)       (((__VALUE__) == USART_DATAWIDTH_8B) \
                                          || ((__VALUE__) == USART_DATAWIDTH_9B))

#define IS_USART_OVERSAMPLING(__VALUE__)    (((__VALUE__) == USART_OVERSAMPLING_16) \
                                          || ((__VALUE__) == USART_OVERSAMPLING_8))

#define IS_USART_STOPBITS(__VALUE__)        (((__VALUE__) == USART_STOPBITS_1) \
                                          || ((__VALUE__) == USART_STOPBITS_2))

USART_RegisterSettings Usart1;

static inline void struct_Control(uint16_t baudrate, uint8_t dataBits, uint8_t parity, uint8_t stopBits) {

    Usart1.BaudRate = baudrate;
    Usart1.DataWidth = dataBits;
    Usart1.Parity = parity;
    Usart1.StopBits = stopBits;

}


/******************************************************************************************
 * @brief Enable Software Flow Control
 * @param USARTn Instance of USART
 * @note  provided for USART_RegisterSettings
 *****************************************************************************************/
inline void enable_UsartSoftwareFlowControl(USART_RegisterSettings* USARTn);

/******************************************************************************************
 * @brief Disable Software Flow Control
 * @param USARTn Instance of USART
 * @note  provided for USART_RegisterSettings
 *****************************************************************************************/
inline void disable_UsartSoftwareFlowControl(USART_RegisterSettings* USARTn);

inline void enable_UsartSoftwareFlowControl(USART_RegisterSettings* USARTn) {
    USARTn->SoftwareFlowControl = 1;
}

inline void disable_UsartSoftwareFlowControl(USART_RegisterSettings* USARTn) {
    USARTn->SoftwareFlowControl = 0;
}


typedef enum
{
  SUCCESS = 0,
  ERROR = !SUCCESS
} ErrorStatus;


/******************************************************************************************
 * @brief Enable Interrups for USART
 * @note  The mode could be NONE, RX, TX OR TX AND RX (Interrupts).
 *****************************************************************************************/
inline void enable_USARTIRQ(USART_IRQMODE modeIrq);

/******************************************************************************************
 * @brief Disable Interrups for USART
 * @note  The mode could be NONE, RX, TX OR TX AND RX (Interrupts).
 *****************************************************************************************/
inline void disable_USARTIRQ(USART_IRQMODE modeIrq);

inline void enable_USARTIRQ(USART_IRQMODE modeIrq) {}
inline void disable_USARTIRQ(USART_IRQMODE modeIrq) {}

typedef struct {

  volatile uint8_t UCSR0A;      // Registro de control A USART0             Address offset: 0x00
  volatile uint8_t UCSR0B;      // Registro de control B USART0             Address offset: 0x01
  volatile uint8_t UCSR0C;      // Registro de control C USART0             Address offset: 0x02
  volatile uint8_t reserved[3]; // Espacio reservado.                       Address offset: 0x03
  volatile uint8_t UDR0;        // Registro de datos USART0                 Address offset: 0x06
  volatile uint16_t UBRR0;      // Registro de velocidad de baudios USART0  Address offset: 0x07

} USART_RegisterMap;

#define USART0_BASE   0xC0                                                // Address of USART0
#define USART1_BASE   0xC8                                                // Address of USART0
#define USART2_BASE   0xD0                                                // Address of USART0
#define USART3_BASE   0x130                                               // Address of USART0

#define USART0        ((USART_RegisterMap *)USART0_BASE)                  // Map Address USART0
#define USART1        ((USART_RegisterMap *)USART1_BASE)                  // Map Address USART0
#define USART2        ((USART_RegisterMap *)USART2_BASE)                  // Map Address USART0
#define USART3        ((USART_RegisterMap *)USART3_BASE)                  // Map Address USART0