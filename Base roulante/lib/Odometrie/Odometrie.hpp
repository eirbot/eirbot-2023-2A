#ifndef odomotrie_h
#define odomotrie_h

#include "math.h"
class Odometrie{
    // Attributs
    int tick_md;
    int tick_mg;
    float x;
    float y;
    float r;
    float theta;
    const int Te_odometrie = 100; // en us  
    const int diametre_roue = 78; // mm
    const float delta_theta = (2 * M_PI) /(14*8);

    // Constructeur
    Odometrie();

    // Méthodes
    void cart2pol(float x, float y);
    void pol2cart(float r, float theta);
    
    void fonction_periodique();

    void set_tick_md(int new_tick_md);
    int get_tick_md();
    void set_tick_mg(int new_tick_mg);
    int get_tick_mg();

    float get_x();
    float get_y();
    float get_r();
    float get_theta();
    void set_x(float new_x);
    void set_y(float new_y);
    void set_r(float new_r);
    void set_theta(float new_theta);
};

#endif