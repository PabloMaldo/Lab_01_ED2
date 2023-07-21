//******************************************************************************
/* 
 * File:   
 * Author: Pablo Maldonado
 * Comments:
 * Revision history: 
 */
//******************************************************************************

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef DISPLAY_H
#define DISLPAY_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>

//******************************************************************************
//  Funcion para mostrar numero en display
//******************************************************************************
char show_display(char number);
//******************************************************************************
//******************************************************************************

//******************************************************************************
//  Funcion para tomar los bits mas significativos 
//******************************************************************************
char upper_bits(char number);
//******************************************************************************
//******************************************************************************

//******************************************************************************
//  Funcion para tomar los bits menos significativos
//******************************************************************************
char lower_bits(char number);
#endif	/* DISPLAY_H */

