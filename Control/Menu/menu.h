#ifndef __MENU_H
#define __MENU_H

typedef enum {
    MENU_MAIN,
    MENU_EDIT
} MenuLevel;

typedef enum {
    ITEM_CIRCLE_COUNT,
    ITEM_BASE_SPEED,
    ITEM_TURN_ANGLE,
    ITEM_COUNT
} MenuItem;

void menu(void);

#endif