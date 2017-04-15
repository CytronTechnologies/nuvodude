/**********************************************************************************
 **********************************************************************************
 ***
 ***    esptool main routine
 ***    - parses the command line parameters and executes the given commands
 ***
 ***    Copyright (C) 2014 Christian Klippel <ck@atelier-klippel.de>
 ***
 ***    This program is free software; you can redistribute it and/or modify
 ***    it under the terms of the GNU General Public License as published by
 ***    the Free Software Foundation; either version 2 of the License, or
 ***    (at your option) any later version.
 ***
 ***    This program is distributed in the hope that it will be useful,
 ***    but WITHOUT ANY WARRANTY; without even the implied warranty of
 ***    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 ***    GNU General Public License for more details.
 ***
 ***    You should have received a copy of the GNU General Public License along
 ***    with this program; if not, write to the Free Software Foundation, Inc.,
 ***    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 ***
 **/

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "infohelper.h"
#include "argparse.h"
#include "nvtcomm.h"

int main(int argc, char **argv)
{
    int num_args;
    int num_args_parsed;
    char **arg_ptr;
    
    num_args = argc-1;
    arg_ptr = argv;
    arg_ptr++;
    
    if(argc < 2)
    {
        LOGERR("No arguments given");
        return 0;
    }
	infohelper_set_infolevel(1);
    infohelper_set_argverbosity(num_args, arg_ptr);
    
    LOGINFO("nuvodude v" VERSION " - (c) 2016 Ng Beng Chet <bengchet@cytron.com.my>");
    
    while(num_args)
    {
        num_args_parsed = parse_arg(num_args, arg_ptr);
        if(num_args_parsed == 0)
        {
            goto EXITERROR;
        }
        
        num_args -= num_args_parsed;
        arg_ptr += num_args_parsed;
    }
	
	if(!nvtcomm_open())
		goto EXITERROR;
	nvtcomm_get_chip_info();
	//nvtcomm_load_file();
	if(nvtcomm_file_ready())
		nvtcomm_upload_file();
    
    //if(nvtcomm_file_uploaded())
		nvtcomm_start_app(1);
	
    return 0;
    
EXITERROR:

    return -1;
}
