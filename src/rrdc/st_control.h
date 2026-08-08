/*
  Hatari — RRDC backend hooks (see rrdc/st_control.c). These three calls are all
  the main loop needs; everything else is the portable core + the backend.
*/

#ifndef HATARI_ST_CONTROL_H
#define HATARI_ST_CONTROL_H

extern void st_control_init(void);    /* startup: read HATARI_CONTROLPORT, bind */
extern void st_control_uninit(void);  /* shutdown: stop the server             */
extern void st_control_vbl(void);     /* per video frame: service + capture + gate */

#endif /* HATARI_ST_CONTROL_H */
