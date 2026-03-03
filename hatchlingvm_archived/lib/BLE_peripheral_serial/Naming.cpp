#include "Naming.h"

char convert_to_ascii(char input); // convenience function

/************************************************************************/
// Check if the three letter word matches the rude words and if yes change the word
/************************************************************************/
int rude_word_check(char *initials_name)
{
	for(unsigned int i=0;i<sizeof(first_letter);i++)
	{
		if(initials_name[0] == first_letter[i] )
		{
			if(initials_name[1] == second_letter[i])
			{
				if(initials_name[2] == third_letter[i] )
				{
					return 1;
				}
			}
		}
	}
	return 0;
}

/************************************************************************/
// Use the micro:bit serial number to find the first three letters of the fancy name to print on the LED screen
// Check if the three letter word matches the rude words and if yes change the word
/************************************************************************/
void getInitials_fancyName(char *initials_name, char macAddr2, char macAddr1, char macAddr0)
{
	int rude_word = 1;

    // Get the last five hex digits of the serial number
    /*uint32_t temp = (microbit_serial_number()&0x000FFFFF);

	uint8_t mod16 = 0;
    uint8_t top8  = 0;
    uint8_t bot6  = 0;
    uint8_t mid6  = 0;
    uint8_t tempCount = 0;*/
    // Use the Mac address to get the name
    //ble_gap_addr_t mac;
    //sd_ble_gap_addr_get(&mac);
    volatile unsigned int temp = 0;
	char mod16 = 0;
    char top8  = 0;
    char bot6  = 0;
    char mid6  = 0;
    char tempCount = 0;
    //Get the Mac Address into a 32 32 bit variable 	
    temp  |= (unsigned int)(macAddr2&0x0F) << 16;	
	temp  |= (unsigned int)macAddr1 << 8;
	temp  |= (unsigned int)macAddr0;

	//temp = (mac.addr[2]&0x0F)*65536+mac.addr[1]*256+mac.addr[0];

	  //Divide 5 bytes into 4 regions 
	mod16  =  temp%16;
	top8   =  temp%256;
	mid6   =  (temp/256)%64;
    bot6   =  (temp/256)/64;
		
		//Use these 4 regions to form 3 letters
	initials_name[0] = name_first[top8 + mod16];
	initials_name[1] = name_second[mid6 + mod16];
	initials_name[2] = name_third[bot6 + mod16];

	initials_name[5] = convert_to_ascii((macAddr2&0x0F));
	
	
	initials_name[6] = (macAddr1&0xF0);
	initials_name[6] = convert_to_ascii(initials_name[6]>>4);
		
	
	initials_name[7] = convert_to_ascii(macAddr1&0x0F);
	initials_name[8] = (macAddr0& 0xF0);
	initials_name[8] = convert_to_ascii(initials_name[8]>>4);
		
	initials_name[9] = convert_to_ascii(macAddr0&0x0F);
		
	//Check if the word generated could be an English rude words
	while(rude_word == 1)
	{
        //check if the existing three letter word is a rude word
        rude_word = rude_word_check(initials_name);
        tempCount++;
        //update the middle letter and see if it is still a rude word - eventually we will get to a non-rude word
        if(rude_word == 1)
        {
            initials_name[0] = name_first[top8 + mod16];
            initials_name[1] = name_second[(mid6 + mod16 + tempCount)%512];
            initials_name[2] = name_third[bot6 + mod16];
        }
    }
}

/************************************************************************/
/**@brief 		Function for converting Hex values to ASCII values
 * @param[in] Input in hex to be converted to ASCII
 * @return 		character which is an ascii value of the input
 */
 /************************************************************************/
char convert_to_ascii(char input)
{
	char output;
	if(input <=9)
	{
		output = input + 0x30;
	}
	else
	{
		output = input + 0x37;
	}
	return output;
}