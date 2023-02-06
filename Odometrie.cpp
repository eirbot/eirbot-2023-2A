#include "Odometrie.hpp"
#include "math.h"

// Constructeur :
Odometrie::Odometrie(){
    r = 0.;
    theta = M_PI_2;
    pol2cart(r, theta);
    tick_md = 0;
    tick_mg = 0;
}

// Get
float Odometrie::get_x(){
    return x;
}
float Odometrie::get_y(){
    return y;
}
float Odometrie::get_r(){
    return r;
}
float Odometrie::get_theta(){
    return theta;
}
int Odometrie::get_tick_md(){
    return tick_md;
}
int Odometrie::get_tick_mg(){
    return tick_mg;
}

// Set
void Odometrie::set_x(float new_x){
    x = new_x;
}
void Odometrie::set_y(float new_y){
    y = new_y;
}
void Odometrie::set_r(float new_r){
    r = new_r;
}
void Odometrie::set_theta(float new_theta){
    theta = new_theta;
}
void Odometrie::set_tick_md(int new_tick_md){
    tick_md = new_tick_md;
}
void Odometrie::set_tick_mg(int new_tick_mg){
    tick_mg = new_tick_mg;
}

// Fonctions
void Odometrie::cart2pol(float x, float y){
    r = sqrt(x * x + y * y);
    if (x != 0){ 
        theta = atan(y/x);
    } else {
        theta = M_PI_2;
    }
}
void Odometrie::pol2cart(float r, float theta){
    x = r * cos(theta);
    y = r * sin(theta);
}

void Odometrie::fonction_periodique(){
    int fixe_tick_md = tick_md;
    int fixe_tick_mg = tick_mg;
    set_tick_md(0);
    set_tick_mg(0);
    r = 0.5 * diametre_roue * delta_theta * (fixe_tick_md + fixe_tick_mg); // m
    r = r / 1000.;
    theta = theta + delta_theta * (fixe_tick_md + fixe_tick_mg); // rad 
    pol2cart(r, theta);
}

