#include <stdio.h>
#include <malloc.h>
#include <string.h>
#include <stdint.h>
#include <libdragon.h>

#define LEFT_MARGIN 10

static volatile uint32_t animcounter = 0;
static resolution_t resolution = RESOLUTION_320x240;
static bitdepth_t depth = DEPTH_16_BPP;
static gamma_t gamma_mode = GAMMA_NONE;
static antialias_t antialias_mode = ANTIALIAS_OFF;
static bool is_high_res_mode = false;

static char *RES_TEXT[6];
static char *GAMMA_TEXT[3];
static char *AA_TEXT[4];

static bool has_changed = false;

void update_counter( int ovfl )
{
    animcounter++;
}

void set_texts()
{
    RES_TEXT[0] = "320x240";
    RES_TEXT[1] = "640x480";
    RES_TEXT[2] = "256x240";
    RES_TEXT[3] = "512x480";
    RES_TEXT[4] = "512x240";
    RES_TEXT[5] = "640x240";

    GAMMA_TEXT[0] = "None";
    GAMMA_TEXT[1] = "Correct";
    GAMMA_TEXT[2] = "Correct Dither";

    AA_TEXT[0] = "Off";
    AA_TEXT[1] = "Resample";
    AA_TEXT[2] = "Fetch Needed";
    AA_TEXT[3] = "Fetch Always";
}

#define NEXT_MODE(value, values_count) \
    has_changed = true; \
    value++; \
    if (value >= values_count) \
    { \
        value = 0; \
    } \


int main(void)
{
    set_texts();

    // enable interrupts (on the CPU) 
    init_interrupts();

    // Initialize peripherals 
    display_init(resolution, depth, 2, gamma_mode, antialias_mode);
    display_toggle_highres_mode(is_high_res_mode);
    dfs_init( DFS_DEFAULT_LOCATION );
    rdp_init();
    controller_init();
    timer_init();

    // Read in single sprite 
    int fp = dfs_open("/mudkip.sprite");
    sprite_t *mudkip = malloc( dfs_size( fp ) );
    dfs_read( mudkip, 1, dfs_size( fp ), fp );
    dfs_close( fp );
    
    fp = dfs_open("/earthbound.sprite");
    sprite_t *earthbound = malloc( dfs_size( fp ) );
    dfs_read( earthbound, 1, dfs_size( fp ), fp );
    dfs_close( fp );

    fp = dfs_open("/plane.sprite");
    sprite_t *plane = malloc( dfs_size( fp ) );
    dfs_read( plane, 1, dfs_size( fp ), fp );
    dfs_close( fp );

    // Kick off animation update timer to fire thirty times a second 
    new_timer(TIMER_TICKS(1000000 / 30), TF_CONTINUOUS, update_counter);

    // Main loop test 
    while(1) 
    {
        if (has_changed)
        {
            display_close();
            display_init(resolution, depth, 2, gamma_mode, antialias_mode);
            has_changed = false;
        }
        static display_context_t disp = 0;

        // Grab a render buffer 
        while( !(disp = display_lock()) );
       
        //Fill the screen 
        graphics_fill_screen( disp, 0xFFFFFFFF );

        // Set the text output color 
        graphics_set_color( 0x0, 0xFFFFFFFF );

        // Hardware spritemap test 
        graphics_draw_text( disp, LEFT_MARGIN + 20, 20, "Graphics Mode Demo" );

        char *text = "";

        sprintf(text, "Resolution: %s", RES_TEXT[resolution]);
        graphics_draw_text( disp, LEFT_MARGIN + 20, 40, text);
        sprintf(text, "Gamma: %s", GAMMA_TEXT[gamma_mode]);
        graphics_draw_text( disp, LEFT_MARGIN + 20, 50, text);
        sprintf(text, "AA: %s", AA_TEXT[antialias_mode]);
        graphics_draw_text( disp, LEFT_MARGIN + 20, 60, text);
        sprintf(text, "High Res Mode: %s", is_high_res_mode ? "On" : "Off");
        graphics_draw_text( disp, LEFT_MARGIN + 20, 70, text);

        // Assure RDP is ready for new commands 
        rdp_sync( SYNC_PIPE );

        // Remove any clipping windows 
        rdp_set_default_clipping();

        // Enable sprite display instead of solid color fill 
        rdp_enable_texture_copy();

        // Attach RDP to display 
        rdp_attach_display( disp );
            
        // Ensure the RDP is ready to receive sprites 
        rdp_sync( SYNC_PIPE );

        // Load the sprite into texture slot 0, at the beginning of memory, without mirroring 
        rdp_load_texture( 0, 0, MIRROR_DISABLED, plane );
        
        // Display a stationary sprite of adequate size to fit in TMEM 
        rdp_draw_sprite( 0, LEFT_MARGIN + 20, 80, MIRROR_DISABLED );

        /* Since the RDP is very very limited in texture memory, we will use the spritemap feature to display
           all four pieces of this sprite individually in order to use the RDP at all */
        for( int i = 0; i < 4; i++ )
        {
            // Ensure the RDP is ready to receive sprites 
            rdp_sync( SYNC_PIPE );

            // Load the sprite into texture slot 0, at the beginning of memory, without mirroring 
            rdp_load_texture_stride( 0, 0, MIRROR_DISABLED, mudkip, i );
        
            // Display a stationary sprite to demonstrate backwards compatibility 
            rdp_draw_sprite( 
                0, 
                LEFT_MARGIN + 50 + (20 * (i % 2)),
                80 + (20 * (i / 2)),
                MIRROR_DISABLED 
            );
        }

        // Ensure the RDP is ready to receive sprites 
        rdp_sync( SYNC_PIPE );

        // Load the sprite into texture slot 0, at the beginning of memory, without mirroring 
        rdp_load_texture_stride( 0, 0, MIRROR_DISABLED, earthbound, ((animcounter / 15) & 1) ? 1: 0 );
        
        // Display walking NESS animation 
        rdp_draw_sprite( 0, LEFT_MARGIN + 20, 120, MIRROR_DISABLED );

        // Ensure the RDP is ready to receive sprites 
        rdp_sync( SYNC_PIPE );

        // Load the sprite into texture slot 0, at the beginning of memory, without mirroring 
        rdp_load_texture_stride( 0, 0, MIRROR_DISABLED, earthbound, ((animcounter / 8) & 0x7) * 2 );
        
        // Display rotating NESS animation 
        rdp_draw_sprite( 0, LEFT_MARGIN + 50, 120, MIRROR_DISABLED );

        // Inform the RDP we are finished drawing and that any pending operations should be flushed 
        rdp_detach_display();

        // Force backbuffer flip 
        display_show(disp);

        // Do we need to switch video displays? 
        controller_scan();
        struct controller_data keys = get_keys_down();

        if(keys.c[0].C_left)
        {
            NEXT_MODE(resolution, 6);
            is_high_res_mode = false;
            display_toggle_highres_mode(is_high_res_mode);
        }

        if (keys.c[0].C_up)
        {
            NEXT_MODE(gamma_mode, 3);
        }

        if (keys.c[0].C_right)
        {
            NEXT_MODE(antialias_mode, 4);
        }

        if (keys.c[0].C_down)
        {
            // Only toggle high_res mode for resolutions that can use it.
            switch (resolution) {
                case RESOLUTION_640x480:
                case RESOLUTION_512x480:
                    is_high_res_mode = !is_high_res_mode;
                    display_toggle_highres_mode(is_high_res_mode);
                    break;
                case RESOLUTION_320x240:
                case RESOLUTION_256x240:
                case RESOLUTION_512x240:
                case RESOLUTION_640x240:
                default:
                    break;
            }
        }
    }
}
