#include <u8g2.h>
#include <mui_u8g2.h>
#include <mui.h>

/*
    Since the screen resolution is 256x128, MUI considers it a bigger display.
    This causes x coordinates to be multiplied by 2
*/

#define INACTIVITY_TIME_S 5     // If no buttons were pressed in this time, UIs go inactive

muif_t muif_list[] = {
    /* Shared objects */
    MUIF_U8G2_LABEL(),
    MUIF_U8G2_FONT_STYLE(0, u8g2_font_helvR08_tf),
    MUIF_U8G2_FONT_STYLE(1, u8g2_font_helvB10_tf),

    /* Main page trip counters */
    MUIF_VARIABLE("TA", NULL, mui_u8g2_btn_goto_wm_fi), // Trip A button
    MUIF_VARIABLE("TB", NULL, mui_u8g2_btn_goto_wm_fi), // Trip B button

    /* Trip A reset */
    MUIF_VARIABLE("AR", NULL, mui_u8g2_btn_goto_wm_fi), // prompt yes response
    MUIF_VARIABLE("AC", NULL, mui_u8g2_btn_back_wm_fi),  // prompt no response

    /* Trip B reset */
    MUIF_VARIABLE("BR", NULL, mui_u8g2_btn_goto_wm_fi), // prompt yes response
    MUIF_VARIABLE("BC", NULL, mui_u8g2_btn_back_wm_fi)  // prompt no response    
};

fds_t fds_data[] = 
/* Main page trip counters */
MUI_FORM(2)         // Trip labels, no selection
MUI_STYLE(0)
MUI_LABEL(96, 58, "Trip A:")
MUI_LABEL(96, 72, "Trip B:")


MUI_FORM(3)         // Trip buttons selectable
MUI_STYLE(0)
MUI_XYAT("TA", 104, 58, 4, "Trip A:")
MUI_XYAT("TB", 104, 72, 6, "Trip B:")

MUI_FORM(4)         // Trip A reset prompt
MUI_STYLE(1)
MUI_LABEL(30, 60, "Reset trip counter A?")
MUI_XYT("AC", 50, 90, "No")
MUI_XYAT("AR", 80, 90, 5, "Yes")

MUI_FORM(6)         // Trip B reset prompt
MUI_STYLE(1)
MUI_LABEL(30, 60, "Reset trip counter B?")
MUI_XYT("BC", 50, 90, "No")
MUI_XYAT("BR", 80, 90, 7, "Yes")

MUI_FORM(5)         // Do trip A reset
MUI_FORM(7)         // Do trip B reset

MUI_FORM(10)        // Config main page
MUI_STYLE(0)

;