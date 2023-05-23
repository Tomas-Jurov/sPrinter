//
// Created by jakub on 22.05.23.
//

#ifndef SRC_PRINTER_STATE_H
#define SRC_PRINTER_STATE_H

enum PrinterState
{
    HOME,
    IDLE,
    FAILURE,
    BUSY,
    INIT,
    PRINTING,
    IDLE2
};

#endif //SRC_PRINTER_STATE_H
