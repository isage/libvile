/**
        libvile
        Copyright (C) 2022 Cat (Ivan Epifanov)

        This program is free software: you can redistribute it and/or modify
        it under the terms of the GNU General Public License as published by
        the Free Software Foundation, either version 3 of the License, or
        (at your option) any later version.

        This program is distributed in the hope that it will be useful,
        but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
        GNU General Public License for more details.

        You should have received a copy of the GNU General Public License
        along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <psp2/kernel/clib.h>
#include <psp2/kernel/threadmgr.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vile.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_mixer.h>

int main(int argc, char *argv[])
{
    vileStart();
    SDL_Init(SDL_INIT_VIDEO|SDL_INIT_JOYSTICK);
    int result = Mix_Init(MIX_INIT_MP3);
    if( result < 0 )
    {
        fprintf(stderr, "Unable to open audio: %s\n", SDL_GetError());
        exit(-1);
    }
    result = Mix_OpenAudio(44100, AUDIO_S16SYS, 2, 512);
    if( result < 0 )
    {
        fprintf(stderr, "Unable to open audio: %s\n", SDL_GetError());
        exit(-1);
    }
    Mix_Music *ymt = Mix_LoadMUS("app0:/data/ymt.mp3");
    if (!ymt)
    {
        fprintf(stderr, "Unable to open audio: %s\n", SDL_GetError());
        exit(-1);
    }

    SDL_Window *window = NULL;
    SDL_Renderer *renderer = NULL;
    SDL_CreateWindowAndRenderer(960, 544, SDL_WINDOW_SHOWN, &window, &renderer);

    SDL_Joystick *joystick;
    joystick = SDL_JoystickOpen(0);

    printf("Name: %s\n", SDL_JoystickNameForIndex(0));

    SDL_SetRenderDrawColor( renderer, 0xFF, 0x00, 0x00, 0xFF );
    SDL_RenderClear( renderer );
    SDL_RenderPresent( renderer );

    while(!vileHasNxt())
    {
        sceKernelDelayThread(1000);
    }

    vileSetInputMode(NXT_IN_2, NXT_SENSOR_SWITCH, NXT_SENSOR_MODE_BOOLEAN);

    SDL_SetRenderDrawColor( renderer, 0x00, 0xFF, 0x00, 0xFF );
    SDL_RenderClear( renderer );
    SDL_RenderPresent( renderer );


    SDL_Event event;
    while (1) {
        SDL_WaitEvent(&event);
        if (event.type == SDL_QUIT) {
            break;
        } else if (event.type == SDL_KEYDOWN) {
            if (event.key.keysym.sym == SDLK_ESCAPE) {
                break;
            }
        } else if (event.type == SDL_JOYAXISMOTION) {
            if (event.jaxis.axis == 1)
            {
                if (event.jaxis.value > 1000 || event.jaxis.value < -1000)
                {
                    vile_setoutputstate_t t = {
                        .port = NXT_OUT_B,
                        .power = event.jaxis.value / 328 / -1.2,
                        .mode = NXT_MOTOR_MODE_ON,
                        .regulation = NXT_MOTOR_REGULATION_SPEED,
                        .turn_ratio = 50,
                        .run_state = NXT_MOTOR_RUNSTATE_RUNNING,
                        .tacho_limit = 0
                    };
                    vileSetOutputState(&t);
                }
                else
                {
                    vile_setoutputstate_t t = {
                        .port = NXT_OUT_B,
                        .power = 0,
                        .mode = NXT_MOTOR_MODE_BRAKE,
                        .regulation = NXT_MOTOR_REGULATION_IDLE,
                        .turn_ratio = 50,
                        .run_state = NXT_MOTOR_RUNSTATE_IDLE,
                        .tacho_limit = 0
                    };
                    vileSetOutputState(&t);
                }
            }
            if (event.jaxis.axis == 3)
            {
                if (event.jaxis.value > 1000 || event.jaxis.value < -1000)
                {
                    vile_setoutputstate_t t = {
                        .port = NXT_OUT_A,
                        .power = event.jaxis.value / 328 / -1.2,
                        .mode = NXT_MOTOR_MODE_ON,
                        .regulation = NXT_MOTOR_REGULATION_SPEED,
                        .turn_ratio = 50,
                        .run_state = NXT_MOTOR_RUNSTATE_RUNNING,
                        .tacho_limit = 0
                    };
                    vileSetOutputState(&t);
                }
                else
                {
                    vile_setoutputstate_t t = {
                        .port = NXT_OUT_A,
                        .power = 0,
                        .mode = NXT_MOTOR_MODE_BRAKE,
                        .regulation = NXT_MOTOR_REGULATION_IDLE,
                        .turn_ratio = 50,
                        .run_state = NXT_MOTOR_RUNSTATE_IDLE,
                        .tacho_limit = 0
                    };
                    vileSetOutputState(&t);
                }
            }
//            printf("axis: %i %i\n", event.jaxis.axis, event.jaxis.value);
        } else if (event.type == SDL_JOYBUTTONDOWN) {
            if (event.jbutton.button == 0) break;
            vilePlayTone( 200*event.jbutton.button, 100);
        }

        vile_inputstate_t st;
        vileGetInputValues(NXT_IN_2, &st);
        if (st.scaled_value == 1)
        {
            Mix_PlayMusic(ymt, 0);
//            break;
        }

        SDL_SetRenderDrawColor( renderer, 0x00, 0xFF, 0x00, 0xFF );
        SDL_RenderClear( renderer );
        SDL_RenderPresent( renderer );
    }

    vile_setoutputstate_t t = {
        .port = NXT_OUT_A,
        .power = 0,
        .mode = NXT_MOTOR_MODE_BRAKE,
        .regulation = NXT_MOTOR_REGULATION_IDLE,
        .turn_ratio = 50,
        .run_state = NXT_MOTOR_RUNSTATE_IDLE,
        .tacho_limit = 0
    };

    vileSetOutputState(&t);

    t.port = NXT_OUT_B;
    vileSetOutputState(&t);


    vileStop();

    SDL_JoystickClose(joystick);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
