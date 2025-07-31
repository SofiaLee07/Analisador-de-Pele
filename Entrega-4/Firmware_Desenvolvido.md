# Firmware 

O objetivo deste projeto foi desenvolver um sistema capaz de realizar a leitura dos parâmetros de oleosidade e hidratação da pele, utilizando sensores analógicos conectados ao microcontrolador MSP430G2553. No entanto, devido a limitações técnicas, a funcionalidade referente à medição de hidratação não pôde ser implementada nesta versão.

Dessa forma, tanto a documentação quanto o código apresentados a seguir tratam exclusivamente da leitura e exibição do nível de oleosidade da pele.

O firmware desenvolvido tem como finalidade ler um sinal analógico proveniente de um sensor de oleosidade, processar esse valor para calcular o percentual de oleosidade, e exibir o resultado em um display OLED SSD1306. O sistema classifica a pele em três estados: seca, normal ou oleosa, com base em faixas definidas de leitura. A exibição é feita de forma clara no display, com atualização periódica controlada por um temporizador interno.

## Main.c

```cpp
#include <msp430.h>
#include <stdint.h>
#include "./lib/i2c_master_g2553.h"
#include "./lib/bits.h"
#include "./lib/gpio.h"
#include "displays/ssd1366.h"

#define LED BIT0
#define LED_PORT P1

volatile uint16_t adc_val = 0;

/**
 * @brief  Configura sistema de clock com calibração.
 */
void init_clock_system(){
#ifdef CLOCK_1MHz
    if (CALBC1_1MHZ==0xFF) while(1);
    DCOCTL = 0;
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL = CALDCO_1MHZ;
#endif
#ifdef CLOCK_8MHz
    if (CALBC1_8MHZ==0xFF) while(1);
    DCOCTL = 0;
    BCSCTL1 = CALBC1_8MHZ;
    DCOCTL = CALDCO_8MHZ;
#endif
#ifdef CLOCK_12MHz
    if (CALBC1_12MHZ==0xFF) while(1);
    DCOCTL = 0;
    BCSCTL1 = CALBC1_12MHZ;
    DCOCTL = CALDCO_12MHZ;
#endif
#ifdef CLOCK_16MHz
    if (CALBC1_16MHZ==0xFF) while(1);
    DCOCTL = 0;
    BCSCTL1 = CALBC1_16MHZ;
    DCOCTL = CALDCO_16MHZ;
#endif

    BCSCTL3 |= LFXT1S_2; // ACLK = VLO
}

/**
 * @brief Configura o ADC10 no canal A2 (P1.2)
 */
void init_adc(){
    ADC10CTL0 = ADC10SHT_2 + ADC10ON + ADC10IE;
    ADC10CTL1 = INCH_2;
    ADC10AE0 |= BIT2;  // Habilita função analógica em P1.2
}

/**
 * @brief  Configura temporizador watchdog como interrupção temporizada.
 */
void config_wd_as_timer(){
    WDTCTL = WDT_ADLY_1000;
    IE1 |= WDTIE;
}
void apagar_linha(oled_partition_t linha) {
    ssd1306_clearDisplay_buffer();
    ssd1306_display_data(linha);
}
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;

    init_clock_system();
    init_i2c_master_mode();
    config_wd_as_timer();
    init_adc();  // <-- Novo

    P1DIR |= BIT0;
    SET_BIT(P1OUT, BIT0);

    __delay_cycles(160000); // Necessario 4 delays
    __delay_cycles(160000);
    __delay_cycles(160000);
    __delay_cycles(160000);

    __bis_SR_register(GIE);

    ssd1306_init();

        apagar_linha(LINE_1);
        apagar_linha(LINE_2);
        apagar_linha(LINE_3);
        apagar_linha(LINE_4);

    while (1){

        ADC10CTL0 |= ENC + ADC10SC;  // Inicia conversão
        __bis_SR_register(CPUOFF + GIE);  // Espera fim da conversão
       
        uint16_t val = adc_val;

        uint8_t oleo_percent;

        // Conversão do valor analógico (0–1023) para percentual de oleosidade (0%–100%)
        // A faixa de entrada é dividida em três zonas para maior sensibilidade perceptiva:
        //
        // 1. Até 55 → faixa SECA: mapeada proporcionalmente para 0% a 25%
        // 2. Entre 56 e 75 → faixa NORMAL: mapeada para 26% a 60%
        // 3. Acima de 75 → faixa OLEOSA: mapeada para 61% a 100%, limitando valor máximo em 100
        //
        // Cada zona aplica uma regra de três para gerar um valor suavizado e escalonado
        // Isso melhora a resolução em regiões críticas da pele, tornando o sistema mais sensível a variações reais
        
        if (val <= 55) {
            oleo_percent = (val * 25) / 55;  // Escala direta de 0–25%
        } else if (val <= 75) {
            oleo_percent = 26 + ((val - 56) * 34) / 19;  // Escala intermediária: 26–60%
        } else {
            uint16_t ajustado = (val > 100) ? 100 : val;  // Limita a 100
            oleo_percent = 61 + ((ajustado - 76) * 39) / 24;  // Escala final: 61–100%
        }

            apagar_linha(LINE_1);
            ssd1306_write_scaled_char(0, 0, 'O', 1);
            ssd1306_write_scaled_char(6, 0, 'L', 1);
            ssd1306_write_scaled_char(12, 0, 'E', 1);
            ssd1306_write_scaled_char(18, 0, 'O', 1);
            ssd1306_write_scaled_char(24, 0, ':', 1);
            ssd1306_write_scaled_char(30, 0, ' ', 1);

            uint8_t cursor = 36;
            uint8_t centenas = oleo_percent / 100;
            uint8_t dezenas = (oleo_percent / 10) % 10;
            uint8_t unidades = oleo_percent % 10;

        if (centenas > 0) {
                ssd1306_write_scaled_char(cursor, 0, '0' + centenas, 1);
                cursor += 6;
            }
            ssd1306_write_scaled_char(cursor, 0, '0' + dezenas, 1);
            cursor += 6;
            ssd1306_write_scaled_char(cursor, 0, '0' + unidades, 1);
            cursor += 6;
            ssd1306_write_scaled_char(cursor, 0, '%', 1);
            ssd1306_display_data(LINE_1);

            ssd1306_write_scaled_char(0, 0, ' ', 2);
            ssd1306_write_scaled_char(16, 0, ' ', 2);
            ssd1306_write_scaled_char(32, 0, 'P', 2);
            ssd1306_write_scaled_char(48, 0, 'E', 2);
            ssd1306_write_scaled_char(64, 0, 'L', 2);
            ssd1306_write_scaled_char(80, 0, 'E', 2);
            ssd1306_write_scaled_char(96, 0, ' ', 2);
            ssd1306_write_scaled_char(112, 0, ' ', 2);
            ssd1306_display_data(LINE_2);


        if (adc_val < 55) {
            // 

            ssd1306_write_scaled_char(0, 0, ' ', 2);
            ssd1306_write_scaled_char(16, 0, ' ', 2);
            ssd1306_write_scaled_char(32, 0, 'S', 2);
            ssd1306_write_scaled_char(48, 0, 'E', 2);
            ssd1306_write_scaled_char(64, 0, 'C', 2);
            ssd1306_write_scaled_char(80, 0, 'A', 2);
            ssd1306_write_scaled_char(96, 0, ' ', 2);
            ssd1306_write_scaled_char(112, 0, ' ', 2);
            ssd1306_display_data(LINE_3);
        }
        else if (adc_val >= 55 && adc_val <= 75) {

            ssd1306_write_scaled_char(0, 0, ' ', 2);
            ssd1306_write_scaled_char(16, 0, 'N', 2);
            ssd1306_write_scaled_char(32, 0, 'O', 2);
            ssd1306_write_scaled_char(48, 0, 'R', 2);
            ssd1306_write_scaled_char(64, 0, 'M', 2);
            ssd1306_write_scaled_char(80, 0, 'A', 2);
            ssd1306_write_scaled_char(96, 0, 'L', 2);
            ssd1306_write_scaled_char(112, 0, ' ', 2);
            ssd1306_display_data(LINE_3);
        }
        else {

            ssd1306_write_scaled_char(0, 0, ' ', 2);
            ssd1306_write_scaled_char(16, 0, 'O', 2);
            ssd1306_write_scaled_char(32, 0, 'L', 2);
            ssd1306_write_scaled_char(48, 0, 'E', 2);
            ssd1306_write_scaled_char(64, 0, 'O', 2);
            ssd1306_write_scaled_char(80, 0, 'S', 2);
            ssd1306_write_scaled_char(96, 0, 'A', 2);
            ssd1306_write_scaled_char(112, 0, ' ', 2); 
            
            ssd1306_display_data(LINE_3);
            }

      

        /* Dorme e acorda com watchdog */
        __bis_SR_register(LPM0_bits + GIE);
    }
}


/* Watchdog timer interrupt */
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=WDT_VECTOR
__interrupt void watchdog_timer(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(WDT_VECTOR))) watchdog_timer (void)
#else
#error Compiler not supported!
#endif
{
    static uint16_t x = 0;
    PORT_OUT(LED_PORT) ^= LED;

    if (x >= 2) {
        x = 0;
        __bic_SR_register_on_exit(CPUOFF);
    }
    x++;
}

/* ADC10 interrupt */
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC10_VECTOR))) ADC10_ISR (void)
#else
#error Compiler not supported!
#endif
{
    adc_val = ADC10MEM;
    __bic_SR_register_on_exit(CPUOFF);
}
```
## Bibliotecas utilizadas 

* msp430.h: Biblioteca padrão fornecida pela Texas Instruments para programação dos microcontroladores da família MSP430.
* stdint.h: Biblioteca que define tipos de dados com tamanho fixo, como uint8_t (8 bits), uint16_t (16 bits), etc.
* i2c_master_g2553.h: Esta biblioteca implementa o protocolo I2C em modo Master para o microcontrolador MSP430G2553. É responsável por permitir a comunicação entre o microcontrolador e o display OLED, transmitindo os dados de texto e comandos de controle necessários.
```cpp
/*
 *  i2c_master_f247_g2xxx.c
 *
 *  Created on: May 01, 2024
 *      Author: Renan Augusto Starke
 *      Instituto Federal de Santa Catarina
 *
 *
 *      - Biblioteca de comunicação I2C em modo Master
 *      - Baseado em msp430g2xx3_usci_i2c_standard_master.c de
 *      Nima Eskandari -- Texas Instruments Inc.
 *
 *                          .   .
 *                         /|\ /|\
 *               CI_xyz    4k7 4k7     MSP430F247
 *              -------     |   |   -------------------
 *             |    SDA|<  -|---+->|P1.7/UCB0SDA       |-
 *             |       |    |      |                   |
 *             |       |    |      |                   |
 *             |       |    |      |                   |
 *             |    SCL|<----+-----|P1.6/UCB0SCL       |
 *              -------            |                   |
 *
 */
/* System includes */
#include <lib/i2c_master_g2553.h>
#include <msp430.h>
#include <stdint.h>
#include <stdlib.h>

/* Project includes */

#if !defined(__MSP430F247__) && !defined(__MSP430G2553__)
    #error "Library no supported/validated in this device."
#endif

struct i2c_status_t {
    /* Used to track the state of the software state machine*/
    i2c_mode state;
    /* Device Addr */
    uint8_t device_addr;
    /* RX: Pointers and index */
    uint8_t *data_to_receive;
    uint8_t rx_byte_count;
    uint8_t rx_index;
    /* TX: Pointers and index */
    uint8_t *data_to_send;
    uint8_t tx_byte_count;
    uint8_t tx_index;
};

/* Estado do módulo I2C */
volatile struct i2c_status_t i2c_status = {0};

void init_i2c_master_mode()
{
    /* Muda P1.6 e P1.7 para modo USCI_B0 */
#if defined(__MSP430F247__)
    P3SEL |= BIT1 + BIT2;
#endif

#if defined(__MSP430G2553__)
    P1SEL |= BIT6 + BIT7;
    P1SEL2|= BIT6 + BIT7;
    /* NÃO HABILITAR Resistores de pull up para o OLED */
#endif

    /* Mantém controlador em reset */
    UCB0CTL1 |= UCSWRST;
    /* I2C Master, synchronous mode */
    UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;
    /* Use SMCLK, keep SW reset */
    UCB0CTL1 = UCSSEL_2 + UCSWRST;

#ifdef CLOCK_16MHz
    /* fSCL = SMCLK/160 = ~100kHz */
   // UCB0BR0 = 80;
    UCB0BR0 = 160;
    UCB0BR1 = 0;
#else
    #error "Clock system not defined for I2C Master Mode support"
#endif
    /* Dummy Slave Address */
    UCB0I2CSA = 0x01;
    /* Clear SW reset, resume operation */
    UCB0CTL1 &= ~UCSWRST;
    UCB0I2CIE |= UCNACKIE;
}


/**
  * @brief  Lê registradores de um dispositivo I2C.
  *         Utiliza IRQ de transmissão para o envio dos bytes.
  *
  *         Use com IRS habilitadas.
  *
  * @param  dev_addr: endereço I2C dos dispositivo.
  *         reg_addr: registrador inicial.
  *         count: número de bytes.
  *         data: vetor onde será armazenado os dados recebidos.
  *
  * @retval i2c_mode: possíveis erros de transmissão.
  */
i2c_mode i2c_master_read_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t count, uint8_t *data)
{
    /* Initialize state machine */
    i2c_status.state = TX_REG_ADDRESS_MODE;
    i2c_status.data_to_receive = data;

    i2c_status.device_addr = reg_addr;
    i2c_status.rx_byte_count = count;
    i2c_status.tx_byte_count = 0;
    i2c_status.rx_index = 0;
    i2c_status.tx_index = 0;

    /* Initialize slave address and interrupts */
    UCB0I2CSA = dev_addr;
    IFG2 &= ~(UCB0TXIFG + UCB0RXIFG);       // Clear any pending interrupts
    IE2 &= ~UCB0RXIE;                       // Disable RX interrupt
    IE2 |= UCB0TXIE;                        // Enable TX interrupt

    UCB0CTL1 |= UCTR + UCTXSTT;             // I2C TX, start condition
    __bis_SR_register(CPUOFF + GIE);              // Enter LPM0 w/ interrupts

    return  i2c_status.state;
}

/**
  * @brief  Escreve um byte no barramento I2C.
  *         Utiliza IRQ de transmissão para o envio do byte.
  *
  *         Use com IRS habilitadas.
  *
  * @param  dev_addr: endereço I2C dos dispositivo.
  *         byte: byte a ser envidado.
  *
  * @retval i2c_mode: possíveis erros de transmissão.
  */
i2c_mode i2c_write_single_byte(uint8_t dev_addr, uint8_t byte){
    return i2c_master_write_reg(dev_addr, byte, NULL, 0);
}


/**
  * @brief  Escreve nos registradores de um dispositivo I2C.
  *         Utiliza IRQ de transmissão para o envio dos bytes.
  *
  *         Use com ISR habilitadas.
  *
  * @param  dev_addr: endereço I2C dos dispositivo.
  *         reg_addr: registrador inicial.
  *         reg_data: dados enviados. Devem permanacer estáticos durante a transmissão.
  *         count: número de bytes.
  *
  * @retval i2c_mode: possíveis erros de transmissão.
  */
i2c_mode i2c_master_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{
    /* Initialize state machine */
    i2c_status.state = TX_REG_ADDRESS_MODE;
    i2c_status.device_addr = reg_addr;
    i2c_status.data_to_send = reg_data;

    /* Use pointers from main:
     *
     * Copy register data to TransmitBuffer
     *
    CopyArray(reg_data, TransmitBuffer, count); */

    i2c_status.tx_byte_count = count;
    i2c_status.rx_byte_count = 0;
    i2c_status.rx_index = 0;
    i2c_status.tx_index = 0;

    /* Initialize slave address and interrupts */
    UCB0I2CSA = dev_addr;
    IFG2 &= ~(UCB0TXIFG + UCB0RXIFG);       // Clear any pending interrupts
    IE2 &= ~UCB0RXIE;                       // Disable RX interrupt
    IE2 |= UCB0TXIE;                        // Enable TX interrupt

    UCB0CTL1 |= UCTR + UCTXSTT;             // I2C TX, start condition

    __bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts: Use no hardware real

    return i2c_status.state;
}


void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count)
{
    uint8_t copyIndex = 0;
    for (copyIndex = 0; copyIndex < count; copyIndex++)
    {
        dest[copyIndex] = source[copyIndex];
    }
}

//******************************************************************************
// I2C Interrupt For Received and Transmitted Data******************************
//******************************************************************************

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0TX_VECTOR))) USCIAB0TX_ISR (void)
#else
#error Compiler not supported!
#endif
{
  if (IFG2 & UCB0RXIFG)                 // Receive Data Interrupt
  {
      //Must read from UCB0RXBUF
      uint8_t rx_val = UCB0RXBUF;

      if (i2c_status.rx_byte_count) {
          if (i2c_status.data_to_receive)
              i2c_status.data_to_receive[i2c_status.rx_index++] = rx_val;
          i2c_status.rx_byte_count--;
      }

      if (i2c_status.rx_byte_count == 1) {
          UCB0CTL1 |= UCTXSTP;
      }
      else if (i2c_status.rx_byte_count == 0) {
          IE2 &= ~UCB0RXIE;
          i2c_status.state = IDLE_MODE;
          __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
      }
  }
  else if (IFG2 & UCB0TXIFG)            // Transmit Data Interrupt
  {
      switch (i2c_status.state)
      {
          case TX_REG_ADDRESS_MODE:
              UCB0TXBUF = i2c_status.device_addr;
              if (i2c_status.rx_byte_count)
                  i2c_status.state = SWITCH_TO_RX_MODE;   // Need to start receiving now
              else
                  i2c_status.state = TX_DATA_MODE;        // Continue to transmision with the data in Transmit Buffer
              break;

          case SWITCH_TO_RX_MODE:
              IE2 |= UCB0RXIE;              // Enable RX interrupt
              IE2 &= ~UCB0TXIE;             // Disable TX interrupt
              UCB0CTL1 &= ~UCTR;            // Switch to receiver
              i2c_status.state = RX_DATA_MODE;    // State state is to receive data
              UCB0CTL1 |= UCTXSTT;          // Send repeated start
              if (i2c_status.rx_byte_count == 1) {
                  //Must send stop since this is the N-1 byte
                  while((UCB0CTL1 & UCTXSTT));
                  UCB0CTL1 |= UCTXSTP;      // Send stop condition
              }
              break;

          case TX_DATA_MODE:
              if (i2c_status.tx_byte_count) {
                  UCB0TXBUF = i2c_status.data_to_send[i2c_status.tx_index++];
                  i2c_status.tx_byte_count--;
              }
              else {
                  //Done with transmission
                  UCB0CTL1 |= UCTXSTP;     // Send stop condition
                  i2c_status.state = IDLE_MODE;
                  IE2 &= ~UCB0TXIE;                       // disable TX interrupt
                  __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
              }
              break;

          default:
              __no_operation();
              break;
      }
  }
}

//******************************************************************************
// I2C Interrupt For Start, Restart, Nack, Stop ********************************
//******************************************************************************

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCIAB0RX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0RX_VECTOR))) USCIAB0RX_ISR (void)
#else
#error Compiler not supported!
#endif
{
    if (UCB0STAT & UCNACKIFG)
    {
        /* Limpa NACK flags, sinaliza NACK e acorda CPU */
        i2c_status.state = NACK_MODE;
        UCB0STAT &= ~UCNACKIFG;
        __bic_SR_register_on_exit(CPUOFF);
    }
    /* Stop or NACK Interrupt */
    if (UCB0STAT & UCSTPIFG)
    {
        /* Limpa START/STOP/NACK Flags */
        UCB0STAT &=  ~(UCSTTIFG + UCSTPIFG + UCNACKIFG);
    }
    if (UCB0STAT & UCSTTIFG)
    {
        /* Limpa START Flags */
        UCB0STAT &= ~(UCSTTIFG);
    }
}
```
* bits.h: Biblioteca para manipulação de bits
```cpp
/*
 * bit.h
 *
 *  Created on: Sep 09, 2016
 *      Author: Renan Augusto Starke
 *      Instituto Federal de Santa Catarina
 *
 *		Bit manipulation macros
 */

#ifndef BITS_H_
#define BITS_H_

#define SET(BIT) (1 << BIT)
#define	SET_BIT(Y,BIT) (Y |= (BIT))
#define	CLR_BIT(Y,BIT) (Y &= ~(BIT))
#define CPL_BIT(Y,BIT) (Y ^= (BIT))
#define TST_BIT(Y,BIT) (Y & (BIT))

#endif /* BITS_H_ */
```

* gpio.h: Biblioteca para controle de GPIO
```cpp
/*
 * gpio.h : GPIO general purpose macros
 *
 *  Created on: Mar 2, 2020
 *      Author: Renan Augusto Starke
 *      Instituto Federal de Santa Catarina
 */

#ifndef LIB_GPIO_H_
#define LIB_GPIO_H_

/* Convert Px to PxOUT */
#define PORT_OUT(...) PORT_OUT_SUB(__VA_ARGS__)
#define PORT_OUT_SUB(port) (port##OUT)

/* Convert Px to PxIN */
#define PORT_IN(...) PORT_IN_SUB(__VA_ARGS__)
#define PORT_IN_SUB(port) (port##IN)

/* Convert Px to PxDIR */
#define PORT_DIR(...) PORT_DIR_SUB(__VA_ARGS__)
#define PORT_DIR_SUB(port) (port##DIR)

/* Convert Px to PxREN */
#define PORT_REN(...) PORT_REN_SUB(__VA_ARGS__)
#define PORT_REN_SUB(port) (port##REN)

/* Convert Px to PxIE */
#define PORT_IE(...) PORT_IE_SUB(__VA_ARGS__)
#define PORT_IE_SUB(port) (port##IE)

/* Convert Px to PxIES */
#define PORT_IES(...) PORT_IES_SUB(__VA_ARGS__)
#define PORT_IES_SUB(port) (port##IES)

/* Convert Px to PxIFG */
#define PORT_IFG(...) PORT_IFG_SUB(__VA_ARGS__)
#define PORT_IFG_SUB(port) (port##IFG)

#endif /* LIB_GPIO_H_ */
```
* ssd1366.c: Arquivo de implementação do driver para o display OLED SSD1306, adaptado para o microcontrolador MSP430. Contém as funções responsáveis por inicializar o display, desenhar caracteres e gráficos, controlar o buffer de exibição e enviar os dados via protocolo I2C.
```cpp
/**
 * @file ssd1366.c
 * @brief Funções de baixo nível para controle do display OLED SSD1306.
 * @author Renan Augusto Starke
 * @date Março 2, 2020
 *
 * Este arquivo contém as implementações das funções para o driver do display OLED SSD1306.
 * As definições de controle do OLED são baseadas em https://github.com/yanbe/ssd1306-esp-idf-i2c.
 * As funções de desenho são baseadas na biblioteca Adafruit_GFX.
 * Algumas definições adicionais são de http://robotcantalk.blogspot.com/2015/03/interfacing-arduino-with-ssd1306-driven.html [2].
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#ifdef __MSP430G2553__
#include "../lib/i2c_master_g2553.h"
#endif


#include "ssd1366.h"
#include "font8x8_basic.h"

/* Following definitions are from:
   http://robotcantalk.blogspot.com/2015/03/interfacing-arduino-with-ssd1306-driven.html
*/

/** @brief Endereço I2C do display OLED. */
#define OLED_I2C_ADDRESS   0x3C

// Configuration data
#define OLED_DISPLAY_WIDTH_PX           0x80
#define OLED_PAGE_HEIGHT_PX             0x08

#define OLED_HEIGHT 64
#define OLED_WIDTH 128

// Control byte
#define OLED_CONTROL_BYTE_CMD_SINGLE    0x80
#define OLED_CONTROL_BYTE_CMD_STREAM    0x00
#define OLED_CONTROL_BYTE_DATA_STREAM   0x40

// Fundamental commands (pg.28)
#define OLED_CMD_SET_CONTRAST           0x81    // follow with 0x7F
#define OLED_CMD_DISPLAY_RAM            0xA4
#define OLED_CMD_DISPLAY_ALLON          0xA5
#define OLED_CMD_DISPLAY_NORMAL         0xA6
#define OLED_CMD_DISPLAY_INVERTED       0xA7
#define OLED_CMD_DISPLAY_OFF            0xAE
#define OLED_CMD_DISPLAY_ON             0xAF
#define OLED_DEACTIVATE_SCROLL          0x2E

// Addressing Command Table (pg.30)
#define OLED_CMD_SET_MEMORY_ADDR_MODE   0x20    // follow with 0x00 = HORZ mode = Behave like a KS108 graphic LCD
#define OLED_CMD_SET_COLUMN_RANGE       0x21    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x7F = COL127
#define OLED_CMD_SET_PAGE_RANGE         0x22    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x07 = PAGE7

// Hardware Config (pg.31)
#define OLED_CMD_SET_DISPLAY_START_LINE 0x40
//#define OLED_CMD_SET_SEGMENT_REMAP      0xA0
#define OLED_CMD_SET_SEGMENT_REMAP      0xA1
#define OLED_CMD_SET_MUX_RATIO          0xA8    // follow with 0x3F = 64 MUX
#define OLED_CMD_SET_COM_SCAN_MODE      0xC8
#define OLED_CMD_SET_DISPLAY_OFFSET     0xD3    // follow with 0x00
#define OLED_CMD_SET_COM_PIN_MAP        0xDA    // follow with 0x12
#define OLED_CMD_NOP                    0xE3    // NOP
#define OLED_SETSTARTLINE               0x40

// Timing and Driving Scheme (pg.32)
#define OLED_CMD_SET_DISPLAY_CLK_DIV    0xD5    // follow with 0x80
#define OLED_CMD_SET_PRECHARGE          0xD9    // follow with 0xF1
#define OLED_CMD_SET_VCOMH_DESELCT      0xDB    // follow with 0x30

// Charge Pump (pg.62)
#define OLED_CMD_SET_CHARGE_PUMP        0x8D    // follow with 0x14


/**
 * @brief Buffer de memória para o display OLED.
 *
 * Este buffer armazena o estado dos pixels antes de serem enviados para o display físico.
 * - Se o microcontrolador for um MSP430G2553, o buffer é particionado em 4
 *   (`(OLED_WIDTH * ((OLED_HEIGHT + 7) / 8))/4`) devido à limitação de RAM [5].
 * - Caso contrário, é alocado um buffer completo de 1KB (`OLED_WIDTH * ((OLED_HEIGHT + 7) / 8)`) [5].
 */
#if defined(__MSP430G2553__)
    /* Not enough RAM for 1k OLED frame Buffer *
     * Using 4 partitions                      */
    uint8_t oled_buffer[(OLED_WIDTH * ((OLED_HEIGHT + 7) / 8))/4];
#else
    /*1k OLED frame Buffer */
    uint8_t oled_buffer[(OLED_WIDTH * ((OLED_HEIGHT + 7) / 8))];
#endif

static void ssd1306_single_command(uint8_t data);
static void ssd1306_command_list(uint8_t *data, uint8_t size);

/**
 * @brief Macro para trocar os valores de duas variáveis int16_t.
 * @param a Primeira variável.
 * @param b Segunda variável.
 */
#ifndef _swap_int16_t
#define _swap_int16_t(a, b)                                                    \
  {                                                                            \
    int16_t t = a;                                                             \
    a = b;                                                                     \
    b = t;                                                                     \
  }
#endif

static const uint8_t init[3][5] = {
    {
        0x00,
        OLED_CMD_DISPLAY_OFF,           // 0xAE
        OLED_CMD_SET_DISPLAY_CLK_DIV,   // 0xD5
        0x80,                           // the suggested ratio 0x80
        OLED_CMD_SET_MUX_RATIO          // 0xA8
    },
    {
        0x00,
        OLED_CMD_SET_DISPLAY_OFFSET,    // 0xD3
        0x0,                            // no offset
        OLED_SETSTARTLINE | 0x0,        // 0x40 line #0
        OLED_CMD_SET_CHARGE_PUMP        // 0x8D
    },
    {
        0x00,
        OLED_CMD_SET_MEMORY_ADDR_MODE,      // 0x20
        0x00,                               // 0x0 act like ks0108
        OLED_CMD_SET_SEGMENT_REMAP | 0x1,   //0xa1
        OLED_CMD_SET_COM_SCAN_MODE,         //0xc8
    },
};

static const uint8_t init_disp_on[] = {
    0x00,
    OLED_CMD_SET_VCOMH_DESELCT, // 0xDB
    0x40,
    OLED_CMD_DISPLAY_RAM, // 0xA4
    OLED_CMD_DISPLAY_NORMAL,       // 0xA6
    OLED_DEACTIVATE_SCROLL,
    OLED_CMD_DISPLAY_ON         // Main screen turn on
};

/**
 * @brief Envia um comando único para o display OLED via I2C.
 * @param data O comando de 8 bits a ser enviado.
 */
static void ssd1306_single_command(uint8_t data){
    i2c_master_write_reg(OLED_I2C_ADDRESS, 0x00, &data, 1);
}

/**
 * @brief Envia uma lista de comandos para o display OLED via I2C.
 * @param data Ponteiro para o array de comandos. O primeiro byte em `data` é o byte de controle (0x00 para stream de comandos),
 *             e os bytes subsequentes são os comandos a serem enviados.
 * @param size O número total de bytes no array `data`.
 */
static void ssd1306_command_list(uint8_t *data, uint8_t size){
    i2c_master_write_reg(OLED_I2C_ADDRESS, data[0], data + 1, size - 1);
}


void ssd1306_init(){
    /* Send all initialization commands */
    ssd1306_command_list((uint8_t *)init[0], sizeof(init[0]));
    ssd1306_single_command(OLED_HEIGHT - 1);
    ssd1306_command_list((uint8_t *)init[1], sizeof(init[1]));
    ssd1306_single_command(0x14);
    ssd1306_command_list((uint8_t *)init[2], sizeof(init[2]));

    ssd1306_single_command(OLED_CMD_SET_COM_PIN_MAP);
    /* Pin MAP must be followed by 0x12*/
    ssd1306_single_command(0x12); 
    ssd1306_single_command(OLED_CMD_SET_CONTRAST);
    /* Contrast  must be followed by 0xCF*/
    ssd1306_single_command(0xCF);  
    ssd1306_single_command(OLED_CMD_SET_PRECHARGE);
    /* Precharge must be followed by 0xF1 */
    ssd1306_single_command(0xF1); 
    ssd1306_command_list((uint8_t *)init_disp_on, sizeof(init_disp_on));

}


/**
 * @brief Limpa o buffer de exibição na memória (RAM) para zeros.
 *
 * Todos os pixels no `oled_buffer` são definidos como 'desligados' (0x00),
 * preparando o buffer para novos desenhos [9].
 */
void ssd1306_clearDisplay_buffer(void) {
    memset(oled_buffer, 0, sizeof(oled_buffer));
}


void ssd1306_draw_pixel(int16_t x, int16_t y, pixel_color_t color){
    if ((x >= 0) && (x < OLED_WIDTH && (y >= 0) && (y < OLED_HEIGHT))) {
        uint16_t i = x + (y >> 3) * OLED_WIDTH;

        if (i > sizeof(oled_buffer) * 4)
            return;

        if (color)
            // oled_buffer[x + (y / 8) * OLED_WIDTH] &= ~(1 << (y & 7));
            //frame_buffer[x + (y >> 3) * OLED_WIDTH] &= ~(1 << (y & 7));
            oled_buffer[i] &= ~(1 << (y & 7));
        else
            //oled_buffer[x + (y / 8) * OLED_WIDTH] |= (1 << (y & 7));
            //frame_buffer[x + (y >> 3) * OLED_WIDTH] |= (1 << (y & 7));
            oled_buffer[i] |= (1 << (y & 7));
    }
}


void ssd1306_display_clear() {
    static const uint8_t cmd[] = {0x00,
                    OLED_CMD_SET_PAGE_RANGE,   // 0x22
                    0, //
                    0xFF,
                    OLED_CMD_SET_COLUMN_RANGE, // 0x21
                    0 };

    ssd1306_command_list((uint8_t *)cmd, sizeof(cmd));
    ssd1306_single_command(OLED_WIDTH - 1);

    uint8_t zero[128];
    memset(zero, 0xff, 128);
    
    uint8_t i;
    for (i = 0; i < 8; i++) {
         i2c_master_write_reg(OLED_I2C_ADDRESS, 0x40, zero, 128);
    }   
}


void ssd1306_display_data(oled_partition_t line){
    uint8_t *data = oled_buffer;
    int i;
    const uint8_t cmd[] = {
        0x00,
        OLED_CMD_SET_PAGE_RANGE,   // 0x22
        (uint8_t) line,  //
        0xFF,
        OLED_CMD_SET_COLUMN_RANGE, // 0x21
        0};

    ssd1306_command_list((uint8_t *)cmd, sizeof(cmd));
    ssd1306_single_command(OLED_WIDTH - 1);

//    for (i=0; i < 1024; i+=128){
//        i2c_master_write_reg(OLED_I2C_ADDRESS, 0x40, data + i, 128);
//    }
#if defined(__MSP430G2553__)
    for (i=0; i < 256; i+=128){
#else
    for (i=0; i < 1024; i+=128) {
#endif
        i2c_master_write_reg(OLED_I2C_ADDRESS, 0x40, data + i, 128);
    }
}


void ssd1306_fill_region(uint8_t x, uint8_t hor_size, uint8_t *data) {
    
    if (x > 8 || hor_size > 128)
    return;

    uint8_t cur_page = x;
    uint8_t init[] = {OLED_CONTROL_BYTE_CMD_STREAM,
                        0x00,
                        0x10,
                        0xB0 | cur_page};

    ssd1306_command_list((uint8_t *)init, sizeof(init));

    i2c_master_write_reg(OLED_I2C_ADDRESS, OLED_CONTROL_BYTE_CMD_STREAM, data, hor_size);
}


void ssd1306_draw_h_line(int16_t x, int16_t y, int16_t size, pixel_color_t color){
    int i;
    for (i=0;i < size;i++)
        ssd1306_draw_pixel(x+i,y,color);
}


void ssd1306_write_char(int16_t x, int16_t y, char data){

  uint8_t *font_ptr = font8x8_basic_tr[(uint8_t)data];

  if ((x >= 0) && (x < (OLED_WIDTH - FONT_PIXEL_WIDTH)) && (y >= 0)
      && (y < (OLED_HEIGHT - FONT_PIXEL_HEIGHT))) {
      
      /* Copy all 8x8 font char data to RAM display buffer */
      memcpy(&oled_buffer[x + (y >>  3) * OLED_WIDTH], font_ptr, 8);
    
  }
}


void ssd1306_writeFastVLine(int16_t x, int16_t y, int16_t h, pixel_color_t color){
    ssd1306_write_line(x, y, x, y + h - 1, color);
}


void ssd1306_fillRect(int16_t x, int16_t y, int16_t w, int16_t h, pixel_color_t color){
    int16_t i;    
    for (i = x; i < x + w; i++) {
        ssd1306_writeFastVLine(i, y, h, color);
  }
}


void ssd1306_write_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, pixel_color_t color){
    int16_t steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) {
        _swap_int16_t(x0, y0);
        _swap_int16_t(x1, y1);
    }

    if (x0 > x1) {
        _swap_int16_t(x0, x1);
        _swap_int16_t(y0, y1);
    }

    int16_t dx, dy;
    dx = x1 - x0;
    dy = abs(y1 - y0);

    int16_t err = dx >> 1;
    int16_t ystep;

    if (y0 < y1) {
        ystep = 1;
    } else {
        ystep = -1;
    }

    for (; x0 <= x1; x0++) {
        if (steep) {
            ssd1306_draw_pixel(y0, x0, color);
        } else {
            ssd1306_draw_pixel(x0, y0, color);
    }
        err -= dy;
        if (err < 0) {
            y0 += ystep;
            err += dx;
        }
    }
}


void ssd1306_write_scaled_char(int16_t x, int16_t y, char data, uint8_t scale){

    int8_t i;
    int8_t j;
    uint8_t *font_ptr =  font8x8_basic_tr[(uint8_t)data];

    for (i = 0; i < 8; i++) {
        uint8_t line = *(font_ptr + i);
        
        for (j = 0; j < 8; j++, line >>= 1) {
            if (line & 1)
                ssd1306_fillRect(x + i * scale, y + j * scale, scale, scale, WHITE_PIXEL);
            else 
                ssd1306_fillRect(x + i * scale, y + j * scale, scale, scale, BLACK_PIXEL);
        }
    }
}
/**
 * @brief Escreve uma string de caracteres no buffer de exibição.
 *
 * Esta função itera sobre a string `data` e chama `ssd1306_write_char`
 * para cada caractere, posicionando-os sequencialmente a cada 8 pixels na coordenada X .
 *
 * @param x Coordenada X de início da string.
 * @param y Coordenada Y de início da string.
 * @param data Ponteiro para a string de caracteres a ser escrita.
 */
void ssd306_write_string(int16_t x, int16_t y, char *data){
    uint16_t i;
    for (i=0; data[i] != '\0'; i++){
        ssd1306_write_char(x + i * 8, y, data[i]);
    }
}
```

* ssd1366.h: Biblioteca do driver do display OLED SSD1306, adaptada para o MSP430.
```cpp
/*
 * ssd1366.c
 *
 *  Created on: Mar 2, 2020
 *      Author: Renan Augusto Starke
 *      Instituto Federal de Santa Catarina *
 *
 *
 *      Definitions from: https://github.com/yanbe/ssd1306-esp-idf-i2c
 *
 */

#ifndef SSD1366_H_
#define SSD1366_H_

#include <stdint.h>

typedef enum  {
    WHITE_PIXEL, BLACK_PIXEL
} pixel_color_t;


/* Quando não há RAM suficiente para o frame Buffer *
 * asa-se 4 partições                               */
typedef enum {
    LINE_1 = 0,     /* First line PAGE_RANGE */
    LINE_2 = 0x32,  /* Second line PAGE_RANGE */
    LINE_3 = 0x64,  /* Third line PAGE_RANGE */
    LINE_4 = 0x96   /* Fourth line PAGE_RANGE */
} oled_partition_t;

void ssd1306_init();

void ssd1306_display_data();
void ssd1306_display_clear();

void ssd1306_fill_region(uint8_t x, uint8_t hor_size, uint8_t *data);

void ssd1306_clearDisplay_buffer(void);

void ssd1306_draw_h_line(int16_t x, int16_t y, int16_t size, pixel_color_t color);
void ssd1306_draw_pixel(int16_t x, int16_t y, pixel_color_t color);

void ssd306_write_string(int16_t x, int16_t y, char *data);
void ssd1306_write_char(int16_t x, int16_t y, char data);
void ssd1306_write_scaled_char(int16_t x, int16_t y, char data, uint8_t scale);

void ssd1306_write_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);

#endif /* SSD1366_H_ */
```



