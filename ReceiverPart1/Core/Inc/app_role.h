#ifndef APP_ROLE_H
#define APP_ROLE_H

/*
 * Change APP_ROLE to APP_ROLE_ACQUISITION when building the sending board.
 * Keep APP_ROLE_RECEIVER for the display board.
 */
#define APP_ROLE_RECEIVER 0
#define APP_ROLE_ACQUISITION 1

#ifndef APP_ROLE
#define APP_ROLE APP_ROLE_RECEIVER
#endif

#endif /* APP_ROLE_H */
