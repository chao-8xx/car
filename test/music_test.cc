#include "headfile.h"


extern Buzzer buzzer;


int main() 
{  

    buzzer.buzzer_init();

    while(1) 
    {
        play_music(qing_tian, sizeof(qing_tian) / sizeof(Note));
        
    }
      
    return 0;
}