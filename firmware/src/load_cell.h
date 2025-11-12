#ifndef LOAD_CELL_H
#define LOAD_CELL_H

extern volatile int wire_tesn;

void load_cell_setup();

void load_cell_tare();

void update_load_cell_value();

#endif
