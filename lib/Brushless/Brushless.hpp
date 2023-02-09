#ifndef brushless_hpp
#define brushless_hpp

class Brushless{
    public:
    int init(void);
    float getVelocity(void);
    

    private:
    float velocity;
    float Kp;
    float Ki; 
    float 
};

#endif