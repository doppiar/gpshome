/***************************
 * Main per applicazione GNSS con u-blox neo 7-P
 * Author:		Giulio Ferrari E3A
 * Copyrighted files - Tesi Bachelor 2018 Elettronici
 * *************************
*/




#include <ublox.h>
#include <usb_unix.h>
#include <gnss_metadata.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <endian.h>
#include <features.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <inttypes.h>

#define UBX_SYNC1 (0xB5)
#define UBX_SYNC2 (0x62)
#define _BSD_SOURCE   

int main(){
	
/*******************************************************************************************/
	/** Ublox or Unix Structure **/

	speedInfo p_speedInfo = {0};


	/** USB VAR **/
	FILE* pUSB0;
	int fdpUSB0;
	char *port0 = "/dev/ttyACM0";
	size_t sizegptxt = 20;
    size_t br;
	char gptxtbuf[sizegptxt];
	int rawbuf[3]={0};			// rawbuf[1] = UBX_SYNC1	rawbuf[2] = UBX_SYNC2	rawbuf[3] = Count many sync
	unsigned char c;

	/** Raw DATA blocks **/
	int classID;
	int ID;
	uint16_t rawmsglen[2]={0};		// length of raw message is 2Bytes
	uint16_t rawmsglen_fin;
	
	signed long rcvTow[4]; 		// Measurment time of week in receiver local time [ms]
	signed long rcvTow_fin=0;
	signed short week[2];		// Measurment week number in receiver local time [weeks]
	signed short week2=0;
	unsigned char numSV;					// Number of satellites following [-]
	unsigned char reserverd1;	// Reserved [-]
	
	uint64_t cpMes[128];			// Carrier phase measurment L1 cycles [cycles] ... 32 is the max number of satellites (better use a pointer and malloc double*numSV)
	uint64_t cpMes_fin[16];		// final array of cpMes where to save shifted results
	uint64_t prMes[128];			// Pseudorange measurment [m]
	uint64_t prMes_fin[16];		// final array of prMes where to save shifted results
	uint32_t doMes[64];			// Dobbler measurement (positive sign for approaching satellites) [Hz]
	uint32_t doMes_fin[16];		// final array of doMes where to save shifted results
	
	unsigned char sv[16];					// Space vehicle number
	signed char mesQI[16];		// Nav Measurment Quality indicatorr: >=4 -> PR+DO OK, >=5 -> PR+DO+CP OK, <6 -> likely loss of carrier look in previous interval
	signed char cno[16];		// signal strngth C/No [dBHz]
	unsigned char lli[16];				// Loss of lock idnicator (RINEX format)
	
	int CK_A;
	int CK_B;


	/** TIME VAR **/
	//int max_ms = 2000;

	/** FLAGS **/
	int gptxtFlag = 0;

	/** FILE VAR **/
	char *file = "../04_SW/RAWDATA.txt";
	FILE* pfile;
	int fdpfile;
	

	/** COUNT VAR **/
	int i=0;
	int k=0;

	/** CONTROL VAR **/
	//int shiftvar=0;

/*******************************************************************************************/	
	/* CTRL+C Quit Rules */
	signal(SIGINT, INThandler);

/*******************************************************************************************/	

	/* Users file */
	pfile = fopen(file, "w");
	if (pfile == NULL) perror ("\nFile RAWDATA.txt Error:\t");
	fdpfile = fileno(pfile);
	if (fdpfile < 0) fprintf(stdout, "Failed to get file descriptor of %s\n", file);
	else{
		fprintf(stdout, "Create/Open %s DONE\n", file);
		fprintf(pfile, "Create/Open %s DONE\n", file);
	}

/*******************************************************************************************/	

	/* USB Setting */
	// Check USB
	slReturn vSdResp = verifySerialDevice(port0);
	if( isErrorReturn( vSdResp ) )
		makeErrorFmtMsgReturn( ERR_CAUSE(vSdResp), "can't find ACM0\n");
	
	//Open the file with name saved in char* port0
	pUSB0  = fopen(port0, "r+");														// r+ = read and write 
	if (pUSB0 == NULL) perror ("Error opening USB0\n");
	// Get the file descriptor
	fdpUSB0 = fileno(pUSB0);
	if (fdpUSB0 < 0 ){
		fprintf(stdout, "Can't get the File Descriptor of %s\n", port0);
		fprintf(stdout, "Error: %s", strerror(errno));
	}
	fprintf(stdout, "The file descriptor's port0 %s is fd = %d\n", port0, fdpUSB0);
	fprintf(pfile, "The file descriptor's port0 %s is fd = %d\n", port0, fdpUSB0);

	usb_unix_init(fdpUSB0, B9600);


	getSpeedInfo(fdpUSB0, &p_speedInfo);

	fprintf(stdout,"BAUDRATE\t%s\tfd=%d:\t%d\n",port0, fdpUSB0, p_speedInfo.baudRate);
	fprintf(stdout,"ns for Bit\t%s\tfd=%d:\t%d\n",port0, fdpUSB0, p_speedInfo.nsBit);
	fprintf(stdout,"ns for Char\t%s\tfd=%d:\t%d\n",port0, fdpUSB0, p_speedInfo.nsChar);

	fprintf(pfile,"BAUDRATE\t%s\tfd=%d:\t%d\n",port0, fdpUSB0, p_speedInfo.baudRate);
	fprintf(pfile,"ns for Bit\t%s\tfd=%d:\t%d\n",port0, fdpUSB0, p_speedInfo.nsBit);
	fprintf(pfile,"ns for Char\t%s\tfd=%d:\t%d\n",port0, fdpUSB0, p_speedInfo.nsChar);



/*******************************************************************************************/	

	/* Once we have the communication from USB we can start to read the Ublox */
		
	fprintf(stdout,"\n READ START\n DATA AVAIABLE HERE:%s\n",file);

	// ------------ READ THE UBX PROTOCOL BEFORE CHANGE THE CODE OR I'LL CUT YOUR HANDS ------------
	// ---------------------------------------------------------------------------------------------
	// ------------------------ Ubx protocol pag. 73, RAW MSG data pag. 174 ------------------------
	// ---------------------------------------------------------------------------------------------
	// https://www.u-blox.com/sites/default/files/products/documents/u-blox7-V14_ReceiverDescriptionProtocolSpec_%28GPS.G7-SW-12001%29_Public.pdf
	
	flushRx(fdpUSB0); //flush junk before read

	while (1)
    {
		
		if(gptxtFlag == 0){
			br = fread(&gptxtbuf, sizegptxt, 1, pUSB0);
			if (br < 0) fprintf(stdout, "Less than 1 byte Read\n");
			for(i = 0; i < sizegptxt; i++){
				fprintf(stdout, "%c", gptxtbuf[i]);
				fprintf(pfile, "%c", gptxtbuf[i]);
				if(gptxtbuf[i-3] == '9' && gptxtbuf[i-2] == '*' && gptxtbuf[i-1] == '2' && gptxtbuf[i] == 'C'){
						gptxtFlag = 1;
					 	fprintf(stdout,"\nNMEA GPTXT Finished\n\n");
					 	i = sizegptxt;
				}
			}
		}
		else{
			
			flushRx(fdpUSB0); //flush junk before read

			/* Now read while c != 0xB5 */
			c = fgetc(pUSB0);
				
			if(c == UBX_SYNC1 /*0xB5*/){
				rawbuf[0] = UBX_SYNC1;
				fprintf(stdout, "\nHeader RAW %d : 0x5B FIND\n", c);
				fprintf(stdout, "-----------------------------------------------\n");
				c = fgetc(pUSB0);
				
				if (c == UBX_SYNC2 /*0x62*/){
					rawbuf[1] = UBX_SYNC2;
					rawbuf[2]++;
					fprintf(stdout, "Header RAW %d: 0x62 FIND\n",c);
					fprintf(stdout, "-----------------------------------------------\n");
					fprintf(stdout, "We are syncronized - RAW DATA COLLECTION BEGIN\n");
					fprintf(stdout, "-----------------------------------------------\n");
					fflush(stdout);
					
					/* Start reading the package: ClassID, ID, Message length */
					classID = fgetc(pUSB0); // should be 0x02	(2 in integer)
					if (classID == 0x02) fprintf(stdout,"\n ClassID UBX 0x02 FIND\n");
					ID = fgetc(pUSB0);		// should be 0x10	(16 in integer)
					if (ID == 0x10) fprintf(stdout,"\n ID UBX-RAW 0x10 FIND\n");
					for(i=0; i<1; i++){
						rawmsglen[i] = fgetc(pUSB0);	// message length of Payload(8byte) + message = 2Byte size
					}

					/* Now we can read the first 8 bytes */
					// that are: 4Byte rcvTom, 2Byte week, 1Byte numSV and reserved1
					for(i=0; i < 3; i++){					
						rcvTow[i] = fgetc(pUSB0);
					}
					for(i=0; i < 1; i++){					
						week[i] = fgetc(pUSB0);
					}
					numSV = fgetc(pUSB0);
					reserverd1 = fgetc(pUSB0);							
					
					/* Now we can start read the RAW DATA*/
					// cpMes 8Byte, prMes 8Byte, doMes 4Bytes, the others four are 1Byte size for a total of tot.byte = 24(*numSV)
					rawmsglen_fin = rawmsglen[0] + (rawmsglen[1]<<8);	// little endian system, the first bit is the LSB 
					for(k=0; k < (rawmsglen_fin-8)/24; k++){

						for(i=(0+k*8); i<(7*(k+1)); i++){
							cpMes[i] = fgetc(pUSB0);						
						}
						for(i=(0+k*8); i<(7*(k+1)); i++){
							prMes[i] = fgetc(pUSB0);						
						}
						for(i=(0+k*8); i<(3*(k+1)); i++){
							doMes[i] = fgetc(pUSB0);						
						}
						sv[k] = fgetc(pUSB0);
						mesQI[k] = fgetc(pUSB0);
						cno[k] = fgetc(pUSB0);
						lli[k] = fgetc(pUSB0);
						
					}

					CK_A = fgetc(pUSB0);
					CK_B = fgetc(pUSB0);

					/* Now we have to elaborate and fix the messages */
					rcvTow_fin = rcvTow[0] + (rcvTow[1]<<8) + (rcvTow[2]<<16) + (rcvTow[3]<<24);
					week2 = week[0] + (week[1]<<8);
					//numSV and reserved are ready to print having only 1 Byte size
					
					for(k=0; k < (rawmsglen_fin-8)/24; k++){

						cpMes_fin[k] = cpMes[0+(k*8)] + (cpMes[1+(k*8)]<<8) + (cpMes[2+(k*8)]<<16) + (cpMes[3+(k*8)]<<24) + (cpMes[4+(k*8)]<<32) + (cpMes[5+(k*8)]<<40) + (cpMes[6+(k*8)]<<48) + (cpMes[7+(k*8)]<<56);
						prMes_fin[k] = prMes[0+(k*8)] + (prMes[1+(k*8)]<<8) + (prMes[2+(k*8)]<<16) + (prMes[3+(k*8)]<<24) + (prMes[4+(k*8)]<<32) + (prMes[5+(k*8)]<<40) + (prMes[6+(k*8)]<<48) + (prMes[7+(k*8)]<<56);		
						doMes_fin[k] = (doMes[0+(k*8)]) + (doMes[1+(k*8)]<<8) + (doMes[2+(k*8)]<<16) + (doMes[3+(k*8)]<<24);	
					}
					
					/* write the result to stdout and RAWDATA.txt */
					fprintf(stdout, "\nClassID: %d\nID: %d\nLength: %d\n", classID, ID, rawmsglen_fin);
					fprintf(stdout, "rcvTow: %ld\t wekk: %d\t nuSV: %d\t reserved: %d\n", rcvTow_fin, week2, (int)numSV, (int)reserverd1);

					for(k=0; k< (rawmsglen_fin-8)/24; k++){
						
						fprintf(stdout,"\n----------------------------\n");
						fprintf(stdout, "--> RAW DATA BLOCK N° %d <--\n", k);

						fprintf(stdout,"\tcpMes: %ld[cycles]\n",cpMes_fin[k]);
						fprintf(stdout,"\tprMes: %ld[m]\n",prMes_fin[k]);
						fprintf(stdout,"\tdoMes: %d[Hz]\n",doMes_fin[k]);

						fprintf(stdout,"\tSV: %d[-]\n",(int)sv[k]);
						fprintf(stdout,"\tmesQI: %d[-]\n",(int)mesQI[k]);
						fprintf(stdout,"\tcno: %d[dBHz]\n",(int)cno[k]);
						fprintf(stdout,"\tlli: %d[-]\n",(int)lli[k]);

					}
		
					fprintf(stdout,"\nCK_A: %d\nCK_B: %d\n", CK_A, CK_B);
					fprintf(stdout, "\n-----------------------------------------------");				
					
					fflush(stdout);

				}
			}				
		}		
	}
		
    fclose (pUSB0);
	fclose (pfile);
	return 0;
	
}

