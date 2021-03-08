#ifndef OVERVIEW_H
#define OVERVIEW_H

typedef struct app_ {

} app_t;

void init_app(app_t *);
void update_app(float dt, app_t *);
void render_app(const app_t *);

#else
#warning "Included overvieew.h twice! ()"
#endif
