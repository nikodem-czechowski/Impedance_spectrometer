#ifdef DEFINE_VARIABLES
#define EXTERN /* nothing */
#else
#define EXTERN extern
#endif /* DEFINE_VARIABLES */


EXTERN char* USB_command; // pointer to command
EXTERN char* USB_parameter; // pointer to command parameter
EXTERN _Bool USB_new_command; // new command from USB to do

EXTERN uint8_t volatile measurement_configuration;  // bits: X X X X [Manual measurement armed][Timer controlled measurement armed] [Trigger 2 armed] [Trigger 1 armed]
EXTERN uint8_t measurements_to_go; 
EXTERN uint16_t number_of_points; 

EXTERN uint16_t start_frequency;
EXTERN uint16_t frequency_increment;

EXTERN uint32_t AD5933_clock; // In Hz

EXTERN uint16_t DS1085_address;
EXTERN uint16_t AD5933_address;

EXTERN uint16_t* measurement_real;
EXTERN uint16_t* measurement_imaginary;
EXTERN uint16_t* frequency;
EXTERN _Bool frequency_set;
EXTERN _Bool measurement_sent;
