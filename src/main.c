#include "stm32f0xx.h"
#include <stdbool.h>

/*
Objective   : 
    - input     → Keyboard
    - output    → Terminal and controlling LED built-in on the PA5

Bugs        : 
    1. sometimes a data cannot show into the terminal (based on UARTbidirectional_v1.1.txt)
    → Solved 
    Solutions
    a.Create a buffer data to solve  loosing data   
    b.Insert interrupt into communication           
    c.Circular Buffer Array 
                              
Notes       :
    - Focus on the bidirectional UART method using the interrupt.
    - Interrupts are managed by the **Nested Vectored Interrupt Controller** (NVIC).
    - In this case, we use the **circular buffers** method to allocate data
*/

#define BUFFER_SIZE 2   //in byte
#define ledPin GPIO_PIN_5

// Define circular buffer structure
typedef struct {
    uint8_t buffer[BUFFER_SIZE];
    volatile uint8_t head; // Index for the next write operation
    volatile uint8_t tail; // Index for the next read operation
    volatile bool full;    // Flag indicating if the buffer is full
} CircularBuffer;

CircularBuffer uart_buffer;
/*
//--Circular Buffer Tools--//
-CircularBuffer_Init
-CircularBuffer_IsEmpty
-CircularBuffer_IsFull
-CircularBuffer_Write
-CircularBuffer_Read
*/

// UART receive function
uint8_t UART_Receive(void) {
    return USART2->RDR; // Read received data
}

// Initialize circular buffer
void CircularBuffer_Init(CircularBuffer *buf) {
    buf->head = 0;
    buf->tail = 0;
    buf->full = false;
}

// Check if buffer is empty
bool CircularBuffer_IsEmpty(CircularBuffer *buf) {
    return (buf->head == buf->tail) && !buf->full;
}

// Check if buffer is full
bool CircularBuffer_IsFull(CircularBuffer *buf) {
    return buf->full;
}

// Add data to buffer or the enqueue operation.
void CircularBuffer_Write(CircularBuffer *buf, uint8_t data) {
    buf->buffer[buf->head] = data;
    buf->head = (buf->head + 1) % BUFFER_SIZE;
    buf->full = (buf->head == buf->tail);
}

// Read data from buffer the dequeue operation.
uint8_t CircularBuffer_Read(CircularBuffer *buf) {
    uint8_t data = buf->buffer[buf->tail];
    buf->tail = (buf->tail + 1) % BUFFER_SIZE;
    buf->full = false;
    return data;
}

// Initialize UART peripheral
void UART_Init(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;    // Enable GPIOA clock
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // Enable USART2 clock

    GPIOA->MODER |= GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1; // PA2 and PA3 to alternate function mode
    GPIOA->AFR[0] |= (1 << 8) | (1 << 12); // Set alternate function for PA2 (USART2_TX) and PA3 (USART2_RX)

    USART2->BRR = SystemCoreClock / 9600; // Set baud rate to 9600
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE; // Enable transmitter, receiver, and USART

    // Enable USART2 interrupts
    NVIC_EnableIRQ(USART2_IRQn);
    USART2->CR1 |= USART_CR1_RXNEIE; // Enable RXNE (Receive not empty) interrupt
    /*
    NVIC_EnableIRQ(USART2_IRQn), 
    the interrupt handler function will be executed automatically when the UART interrupt occurs.
    */
}

// UART transmit function
void UART_Transmit(uint8_t data) {
    /*
    TXE
    0: data is not transferred to the shift register
    1: data is transferred to the shift register)
    */
    while (!(USART2->ISR & USART_ISR_TXE)); // Wait until TXE data registeris empty or 0
    USART2->TDR = data; // Send data
}

// UART receive interrupt handler
void USART2_IRQHandler(void) {
    /*
    RXNE
    0: data is not received
    1: Received data is ready to be read.
    */
    // Check if RXNE (Data register isn't empty) flag is set
    if (USART2->ISR & USART_ISR_RXNE) { 
        uint8_t received_data = UART_Receive(); // Read received data
        CircularBuffer_Write(&uart_buffer, received_data); // Write received data to buffer
    }
}

void GPIOinit(){
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN|RCC_AHBENR_GPIOCEN;    // enable GPIO clock
    /*configure GPIO pin as ouptut on A5 and input on C13*/ 
    GPIOA->MODER |= GPIO_MODER_MODER5_0 ;   //general purpose output mode on pin A5
    GPIOC->MODER &= ~(GPIO_MODER_MODER13);  //set pin 13 as an input
    
    /*enable floating up C13*/
    GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR13);   //reset PC13 into floating mode
}

// Main function
int main(void) {

    // casting integer value to a pointer
    uint8_t data, *dataPtr = (uint8_t*)&data;
    // Initialize GPIO
    GPIOinit();
    // Initialize peripherals
    CircularBuffer_Init(&uart_buffer);
    UART_Init();

    while (1) {
        //check if the buffer is not empty, it reads data on the buffer
        if (!CircularBuffer_IsEmpty(&uart_buffer)) {
            *dataPtr = CircularBuffer_Read(&uart_buffer); // Read data from buffer
            UART_Transmit(*dataPtr); // Transmit data
        }

        //execute led based in the data input
        if (*dataPtr=='a'){
            GPIOA->BSRR |= GPIO_BSRR_BS_5;
        }
        else{
            GPIOA->BSRR |= GPIO_BSRR_BR_5;
        }
    }
}
