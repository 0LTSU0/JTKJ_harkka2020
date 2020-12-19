/*
K�YTT�OHJEET:

Valikossa ylempi nappi siirt�� kursoria, alempi valitsee.

Pelin ollessa k�ynniss� ylemm�ll� napilla voidaan siirty� satunnaiseen ruutuun. Alempi nappi taas peruuttaa yhden siirron takaisin.
Komentojen ketjutusta en valitettavasti kerennyt toteuttaan jonka takia hirve�ll� vauhdilla (niin napilla kuin liikkeell�) ei komentoja kannata antaa.
T�ll� nimitt�in mit� todenn�k�isimmin saa aikaan mit� mielenkiintoisimpia komentoja tai jopa Sensortagin jumahtamisen. K�yt�nn�ss� jokaisen komennon
j�lkeen kannattaa odottaa n�yt�n r�ps�hdyst�. T�m� kertoo laitteen olevan valmis suraavaan siirtoon. T�h�n samaan liittyen t�ytyy mainita er��st� bugista,
jota en onnistunut korjamaan ilman ett� kaikki muu hajosi: Siirrytt�ess� takaisin tai satunnaiseen suuntaan, rekister�idyn komennon n�ytt�misen j�lkeen
n�yt�ll� vilahtaa teksti "komentoa ei tunnistettu". Lyhyt versio t�st� on se, ett� taustalla oleva datankeruu py�rii koko ajan ja laitteen ollessa paikallaan
nappia k�tett�ess� liikedatasta ei tunnisteta mit��n, joka saa tuon tekstin vilahtamaan. T�m� ei sin�ns� vaikuta laitteen toimintaan eik� k�ytt�miseen ylip��ns�,
joten kannattaa vain ignorata se :).

Toinen vaihtoehto menussa on edellisen pelin siirtojen katsominen. T�m� yksinkertaisesti vain tulostaa n�yt�lle edellisen pelin siirroista. Jos siirtoja
ennem�n kuin mit� n�yt�lle mahtuu, voi alemmalla napilla kelata listaa alas. Ylempi nappi palauttaa t�ss� takaisin menuun.

Vaihtoehto "reset" nimens� mukaisesti resetoi laitteen nollaamalla kaikki siirtohistoriaan liittyv�t jutut. T�t� kannattaa painaa ennen uuden pelin aloittamista
ellei halua peruutusfunktion peruuttelevan edellisen pelin siirtoja :). T�t� reset�intifunktiotahan voisi muuten kutsua automaattisesti pelin loppuessa, mutta
t�ll�in edellisen pelin siirtoja ei p��sisi katsomaa. Uuden pelin alkaessa t�m�n kutsuminen ei olisi ongelma, mutta koska menu oli kahdella vaihtoehdolla
melko tyls�n ja tyhj�n n�k�inen, halusin j�tt�� t�m�n manuaaliseksi jutuksi.

Er�s asia josta my�s haluan mainita on, ett� jos debuggauksen aloitus ei onnistu ja USB:n joutuu k�ytt��n irti, l�hes joka kerta ohjelmaa suorittaessa alkaa
laite syyt�m��n system_printf:ll� jotain read erroria. En ole ihan varma aiheutinko itse t�m�n vai johtuuko jostain kirjastosta, mutta varoitampahan kuitenkin.
*/


#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>

#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <time.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/mw/display/Display.h>
#include <ti/mw/display/DisplayExt.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/i2c/I2CCC26XX.h>
#include <ti/mw/remotecontrol/buzzer.h>

/* Board Header files */
#include "Board.h"
#include "wireless/comm_lib.h"
#include "sensors/mpu9250.h"

//Taskit
#define STACKSIZE 2048
Char sensorTaskStack[STACKSIZE];
Char displayTaskStack[STACKSIZE];
Char commTaskStack[STACKSIZE];

//MPU:n jutut
static PIN_Handle hMpuPin;
static PIN_State MpuPinState;
static PIN_Config MpuPinConfig[] = {
    Board_MPU_POWER  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

static const I2CCC26XX_I2CPinCfg i2cMPUCfg = {
    .pinSDA = Board_I2C0_SDA1,
    .pinSCL = Board_I2C0_SCL1
};


//nappien handlet ja statet
static PIN_Handle buttonHandle;
static PIN_State buttonState;
static PIN_Handle buttonHandle2;
static PIN_State buttonState2;

//Ledien vastaavat
static PIN_Handle ledHandle;
static PIN_State ledState;
static PIN_Handle led2Handle;
static PIN_State led2State;

//buzzerin handle ja state
static PIN_Handle BuzzerHandle;
static PIN_State BuzzerState;


//PIN_Configit buzzerille, napeille ja ledeille
static PIN_Config Buzzer[] = {
    Board_BUZZER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

PIN_Config buttonConfig[] = {
   Board_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE, // Nelj�n vakion TAI-operaatio
   PIN_TERMINATE // Taulukko lopetetaan aina t�ll� vakiolla
};

PIN_Config buttonConfig2[] = {
   Board_BUTTON1  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE, // Nelj�n vakion TAI-operaatio
   PIN_TERMINATE // Taulukko lopetetaan aina t�ll� vakiolla
};

PIN_Config ledConfig[] = {
   Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
   PIN_TERMINATE // Taulukko lopetetaan aina t�ll� vakiolla
};

PIN_Config led2Config[] = {
   Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
   PIN_TERMINATE // Taulukko lopetetaan aina t�ll� vakiolla
};



//Muuttujat jotka useassa paikassa k�yt�ss�:
float ax[10], ay[10], az[10], gx[10], gy[10], gz[10]; //ker�tty data sij. n�ihin
int index[10]; //datan indeksi (Clock_getTicks)
int alotusaika = 0; //Muistetaan pelin aloitusaika
int peliaika = 0; //pelin aloitusaika - lopetusaika
int siirrot; //Siirtojen lkm.
int uudetsiirrot = 0; //Ns. uusien siirtojen lkm. k�ytet��n peruuttamisessa (tarkemmin toiminta sen toiminnallisuuden kohdalla)
int kelaus = 0; //Ei nyky��n k�yt�ss�, mutta en poista jos joudunkin ottaan vanhan funktion k�ytt��n jostain syyst�.
int noice = 0; //1=vierityksen ��ni, 2=valinnan ��ni, 0=ei mtn.
char *komentohistoria[40] = {""}; //Tallennetaan siirrot ihmiselle luettavaan muotoon (MAX 40 siirtoa)
int komentohistorianrot[40]; //Siirtojen historia numeroina 1-4 (ohjelma k�ytt�� n�it� "koodeja") 1=UP, 2=DOWN, 3=RIGHT ja 4=LEFT
char komento[6] = ""; //komento viestin l�hetyst� varten
int vieritys=0; //edellisen pelin siirtoja katsoessa k�ytet��n vieritt��n listaa, jos enemm�n komentoja kuin n�yt�lle mahtuu
char lopputulos[20] = "";
int tila = 5; //1=lukee datoja, 2=analysoi, 3=n�yt�lle uus liike, 5=menu, 6=v�liaikainen(tyhjent�� n�yt�n jossain tilanteissa), 7=n�yt� edellinen peli, 8=odotustila eli ts. ei tehd� mit��n ennen jotain inputtia, 10=Kommunikointi, 11=WIN, 12=LOST

//Erin�isi� "kyll�/ei" muuttujia, joilla muistetaan onko tietyt asiat k�ynniss� yms. k�ytet��n rajoittamaan suoritusta tietyiss� kohdissa
int sensori_valmis = 0; //0=ei, 1=on
int pelimenossa = 0; //0=ei, 1=on, 2=k�yt�nn�ss� sama kuin 1, mutta k�ytet��n n�yt�lle
int halutaanedellinen = 0; //tieto analysointifunktiolle, ett� pit��k� suorittaakkin edellinen komento

Display_Handle displayHandle;

int kursori = 0; //main menun kursorin sijainti
int komentotunnistettu = 0; //tunnistettu komento numerokoodina ks. yll�
char menu[3][16] = {"Aloita", "Edellinen", "Reset"};

//prototyypit:
//void analysointi();
void analysointi2();
void valitsemisfunktio();
void edellinen();
void tulostahistoria();
void edellinen2();
void resettifunktio();
void random();
void commTaskFxn(UArg arg0, UArg arg1);


//N�yt�n taski joka todellisuudessa my�s esimerkiksi hoitaa ��nitehosteet
Void displayTaskFxn(UArg arg0, UArg arg1){
	//N�YTT� VALMIIKSI
	Display_Params params;
	Display_Params_init(&params);
	params.lineClearMode = DISPLAY_CLEAR_BOTH;

	Display_Handle displayHandle = Display_open(Display_Type_LCD, &params);
	System_printf("N�ytt� valmis");
	System_flush();
	//Kerrotaan k�ytt�j�lle, ett� kalibrointi menossa:
	Display_print0(displayHandle, 0, 0, "Kaynnistyy ja");
	Display_print0(displayHandle, 1, 0, "kalibroituu!");
	Display_print0(displayHandle, 3, 0, "Ela liikuta!");

	tContext *pContext = DisplayExt_getGrlibContext(displayHandle);

	//muuttujia joita n�yt�lle tulostetaan
	int p;
	char cursor[3] = ">";
	char valinta[16] = "";
	char siirtomaarateksti[16];
	while(1){
		Task_sleep(20000 / Clock_tickPeriod);

		//Kun kalibrointi valmis, poistetaan n�yt�lt� siit� kertovat tekstit ja siirryt��n menutilaan
		if(tila==6 && sensori_valmis == 1){
			Display_clear(displayHandle);
			tila = 5;
		}

		//Main menu; "Ylempi" nappi vieritt�� menua, "alemmalla" valitaan
		while(tila==5 && sensori_valmis == 1){
			Display_print0(displayHandle, 0, 0, "Main menu:");
			for(p=0;p<3;p++){
				if(p==kursori){
					sprintf(valinta, "%s%s", cursor, menu[p]);
					Display_print0(displayHandle, 2+p, 1, valinta);
				}
				else{
					Display_print0(displayHandle, 2+p, 2, menu[p]);
				}
			}

			//Nappia painaessa molemmille omat ��net
			if(noice == 1){
				buzzerSetFrequency(440);
				Task_sleep(12000 / Clock_tickPeriod);
				buzzerSetFrequency(0);
				noice = 0;
			}
			if(noice == 2){
				buzzerSetFrequency(660);
				Task_sleep(12000 / Clock_tickPeriod);
				buzzerSetFrequency(0);
				noice = 0;
			}
		}


		//kun peli aloitetaan, tyhjennet��n n�ytt�
		if(tila==6 && pelimenossa == 1){
			Display_clear(displayHandle);
			tila=1;
			Task_sleep(5000 / Clock_tickPeriod);
			pelimenossa =2;
		}

		//Perusasiat mitk� aina n�yt�ll� n�kyviss� pelin ollessa k�ynniss�
		if(tila==1 && pelimenossa == 2){
			Display_print0(displayHandle, 1, 1, "Peli kaynnissa!");
			Display_print0(displayHandle, 3, 1, "Komento:");
			sprintf(siirtomaarateksti, "Siirtoja: %i", siirrot);

			//System_printf(siirtomaarateksti);
			//System_flush();

			Display_print0(displayHandle, 11, 0, siirtomaarateksti);
			Task_sleep(5000 / Clock_tickPeriod);
			peliaika = Clock_getTicks() - alotusaika;
			//System_printf("%i\n", peliaika);
			//System_flush();
		}

		//Tulkitusta siirrosta riippuen piirret��n n�yt�lle nuoli + teksti:
		tContext *pContext = DisplayExt_getGrlibContext(displayHandle);
		if(tila==3 && pelimenossa == 2){
			if(komentotunnistettu == 1){
				Display_print0(displayHandle, 4, 1, "Ylos");
				GrLineDraw(pContext,40,50,40,85);
				GrLineDraw(pContext,56,50,56,85);
				GrLineDraw(pContext,40,85,56,85);
				GrLineDraw(pContext,30,50,66,50);
				GrLineDraw(pContext,30,50,48,40);
				GrLineDraw(pContext,66,50,48,40);
				GrFlush(pContext);
			}
			else if(komentotunnistettu == 2){
				Display_print0(displayHandle, 4, 1, "Alas");
				GrLineDraw(pContext,40,40,40,75);
				GrLineDraw(pContext,56,40,56,75);
				GrLineDraw(pContext,40,40,56,40);
				GrLineDraw(pContext,30,75,66,75);
				GrLineDraw(pContext,30,75,48,85);
				GrLineDraw(pContext,66,75,48,85);
				GrFlush(pContext);
			}
			else if(komentotunnistettu == 3){
				Display_print0(displayHandle, 4, 1, "Oikealle");
				GrLineDraw(pContext,20,56,65,56); //
				GrLineDraw(pContext,20,72,65,72);//
				GrLineDraw(pContext,65,80,75,64);
				GrLineDraw(pContext,65,48,75,64);
				GrLineDraw(pContext,65,80,65,48);//
				GrLineDraw(pContext,20,72,20,56);//
				GrFlush(pContext);
			}
			else if(komentotunnistettu == 4){
				Display_print0(displayHandle, 4, 1, "Vasemmalle");
				GrLineDraw(pContext,30,56,75,56);
				GrLineDraw(pContext,30,72,75,72);
				GrLineDraw(pContext,20,64,30,80);
				GrLineDraw(pContext,20,64,30,48);
				GrLineDraw(pContext,30,80,30,48);
				GrLineDraw(pContext,75,72,75,56);
				GrFlush(pContext);
			}
			else if(komentotunnistettu == 0){
				Display_print0(displayHandle, 3, 1, "Komentoa");
				Display_print0(displayHandle, 4, 1, "ei tunnistettu!");
			}

			//Jos komento tunnistetaan, ilmoitetaan my�s ��nimerkill�
			if(komentotunnistettu != 0){
				buzzerSetFrequency(660);
				Task_sleep(12000 / Clock_tickPeriod);
				buzzerSetFrequency(0);
				noice = 0;
			}
			Task_sleep(988000 / Clock_tickPeriod);

			Display_clear(displayHandle); //T�m� tarkoituksella r�ps�ytt�� n�ytt�� kertoen laitteen olevan valmis seuraavaan siirtoon.

			Display_print0(displayHandle, 1, 1, "Peli kaynnissa!");
			Display_print0(displayHandle, 3, 1, "Komento:");

			sprintf(siirtomaarateksti, "Siirtoja: %i", siirrot);
			Display_print0(displayHandle, 11, 0, siirtomaarateksti);
			tila=10; //Vaihdetaan tila 10:n jossa commtaski l�hett�� viestin
		}

		//Pelin loppuessa tavalla tai toisella kerrotaan tekstill� ja ledeill� k�ytt�j�lle t�st�:
		if(tila == 11 || tila == 12){
			Display_clear(displayHandle);
			if(tila == 11){
				Display_print0(displayHandle, 1, 1, "Voitit!!!");
				Display_print0(displayHandle, 3, 1, siirtomaarateksti);
				PIN_setOutputValue( ledHandle, Board_LED0, !PIN_getOutputValue( Board_LED0 ) );
				Task_sleep(2000000 / Clock_tickPeriod);
				PIN_setOutputValue( ledHandle, Board_LED0, !PIN_getOutputValue( Board_LED0 ) );

			}
			else if(tila == 12){
				Display_print0(displayHandle, 1, 1, "Havisit :C");
				Display_print0(displayHandle, 3, 1, siirtomaarateksti);
				PIN_setOutputValue( led2Handle, Board_LED1, !PIN_getOutputValue( Board_LED1 ) );
				Task_sleep(2000000 / Clock_tickPeriod);
				PIN_setOutputValue( led2Handle, Board_LED1, !PIN_getOutputValue( Board_LED1 ) );
			}
			Display_clear(displayHandle);
			tila = 5; //palataan menuun
		}

		//Edellisen pelin siirtojen katsominen:
		while(tila == 7){
			int j;
			int i = 1;
			char rivi[40];
			Display_clear(displayHandle);
			Display_print0(displayHandle, 0, 0, "Edellinen peli:");
			if(siirrot > 0){
				for(j=0+vieritys;j<11+vieritys;j++){
					sprintf(rivi, "%i. %s", j+1, komentohistoria[j]);
					Display_print0(displayHandle, i, 0, rivi);
					i++;
					//System_printf("%s\n", komentohistoria[j]);
					//System_flush();
				}
				i=1;
				tila=8;

			}
			else{tila=5;}
		}
	} //whilen loppu

}



Void sensorTaskFxn(UArg arg0, UArg arg1) {

	//Sensori valmiiksi:
	I2C_Handle i2cMPU; // INTERFACE FOR MPU9250 SENSOR
	I2C_Params i2cMPUParams;
	I2C_Params_init(&i2cMPUParams);
	i2cMPUParams.bitRate = I2C_400kHz;
	i2cMPUParams.custom = (uintptr_t)&i2cMPUCfg;

	i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
	if (i2cMPU == NULL) {
	    System_abort("Error Initializing I2CMPU\n");
	}

	PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);
	Task_sleep(100000 / Clock_tickPeriod);
    System_printf("MPU9250: Power ON\n");
    System_flush();

    System_printf("MPU9250: Setup and calibration...\n");
    System_flush();

    mpu9250_setup(&i2cMPU);

    System_printf("MPU9250: Setup and calibration OK\n");
    System_flush();
    sensori_valmis = 1;
    tila = 6;
    //I2C_close(i2cMPU);
    int i;
    //char arvot[] = "";
    //int j;
    halutaanedellinen = 0;
    while(1){
    	if(tila == 6 || tila == 5 || tila == 3 || tila == 7 || tila == 8 || tila == 11 || tila == 12){
    		Task_sleep(8000 / Clock_tickPeriod);
    	}
    	if(tila == 1){
    		//i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
    		//    if (i2cMPU == NULL) {
    		//    	System_abort("Error Initializing I2CMPU\n");
    		//    }
    		Task_sleep(50 / Clock_tickPeriod);
    		halutaanedellinen = 0;

    		//Ker�t��n 10 kpl datapisteit�
    		for(i=0;i<10;i++){
    			//i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
    			mpu9250_get_data(&i2cMPU, &ax[i], &ay[i], &az[i], &gx[i], &gy[i], &gz[i]);
    			index[i] = Clock_getTicks();
    			//I2C_close(i2cMPU);
    			//sprintf(arvot, "%i, %f, %f, %f   %f, %f, %f\n", index[i], ax[i], ay[i], az[i], gx[i], gy[i], gz[i]);
    		    //System_printf(arvot);
    		    //System_flush();
    		    Task_sleep(200000 / Clock_tickPeriod);
    		    if(i == 9){
		    		System_printf("\nNYT\n");
		    		System_flush();
		    		tila=3;
		    		analysointi2();
		    		Task_sleep(500);
    		    }
    		//I2C_close(i2cMPU);
    	}

    	//Task_sleep(400000 / Clock_tickPeriod);
    }
}
}

float max_gx;
float min_gx;
float max_gy;
float min_gy;
int max_gx_index;
int min_gx_index;
int max_gy_index;
int min_gy_index;
//char komento[6] = "";
int k;
void analysointi2(){
	komentotunnistettu = 0; //tunnistettu komento nollaksi silt� varalta ett� ei liikett�
	max_gx = gx[0];
	max_gy = gy[0];

	//Etsit��n max arvot gy ja gx sek� otetaan talteen niiden indeksit
	for(k=0;k<10;k++){
		if(gx[k]>max_gx){
			max_gx = gx[k];
			max_gx_index = index[k];
		}
		if(gy[k]>max_gy){
				max_gy = gy[k];
				max_gy_index = index[k];
		}
	}

	//Etsit��n min avot gy ja gx:lle sek� otetaan my�s niiden indeksit talteen
	min_gx = gx[0];
	min_gy = gy[0];
	for(k=0;k<10;k++){
		if(gy[k]<min_gy){
			min_gy=gy[k];
			min_gy_index = index[k];
		}
		if(gx[k]<min_gx){
			min_gx = gx[k];
			min_gx_index = index[k];
		}
	}

	//analysoidaan etsityill� arvoilla tarkoitettu liike raja-arvo 75 tuntui testauksen perusteella olevan sopiva. t�ll�in liikkeen pit�� olla melko selv� eik� virhetulkintoja juurikaan tapahdu:
	sprintf(komento, "");
	if(max_gx_index < min_gx_index && max_gx>75 && min_gx<-75){
		sprintf(komento, "UP");
		komentotunnistettu = 1; //1=UP
		if(siirrot < 40) {
			komentohistoria[siirrot] = "Ylos";
			komentohistorianrot[siirrot] = 1;
		}
	}
	if(min_gx_index < max_gx_index && max_gx>75 && min_gx<-75){
		sprintf(komento, "DOWN");
		komentotunnistettu = 2; //2=DOWN
		if(siirrot < 40) {
			komentohistoria[siirrot] = "Alas";
			komentohistorianrot[siirrot] = 2;
		}
	}
	if(min_gy_index > max_gy_index && min_gy<-75 && max_gy>75){
		sprintf(komento, "RIGHT");
		komentotunnistettu = 3; //3=RIGHT
		if(siirrot < 40) {
			komentohistoria[siirrot] = "Oikea";
			komentohistorianrot[siirrot] = 3;
		}
	}
	if(max_gy_index > min_gy_index && min_gy<-75 && max_gy>75){
		sprintf(komento, "LEFT");
		komentotunnistettu = 4; //4=LEFT
		if(siirrot < 40){
			komentohistoria[siirrot] = "Vasen";
			komentohistorianrot[siirrot] = 4;
		}
	}

	//T�ss� tulee vastaan tuo uudetsiirrot homma. Jos komento tulkitaan liikkeest�, laitetaan uudetsiirrot=siirrot, jolloin taaksepp�in kelaaminen
	//alkaa tavallaan viimeisimm�st� liikkeest�. Jos uutta liikett� ei tule, pysyy uudetsiirrot entisell��n jolloin pakittaminen jatkuu siit�
	//mihin asti on pakitettu
	if(komentotunnistettu != 0){
		siirrot++;
		uudetsiirrot = siirrot;
		kelaus=0;
	}
	System_printf(komento);
	System_flush();
	//tulostahistoria();
}
	//tila=3; //pakotetaan n�yt�n p�ivits

char sanat[100]; //Debugauksessa k�ytetty funktio jolla tulostettiin historia. Ei k�yt�ss� varsinaisessa ohjelmassa.
char numerot[50];
int i;
void tulostahistoria(){
	for(i=0;i<siirrot;i++){
		sprintf(sanat,"%s\n",komentohistoria[i]);
		sprintf(numerot,"%i\n",komentohistorianrot[i]);
		System_printf(sanat);
		System_flush();
		System_printf(numerot);
		System_flush();
	}


}



char sanaa[];
//Pakitusta pyydett�ess� t�t� funktiota kutsutaan
void edellinen2(){
	//Jos ei ole yht��n siirtoa mit� pakittaa, ei tehd� mtn. Jos taas pakitus on mahdollinen, samalla tavalla kuin analysointifunktiossa
	//laitetaan tapahtuva siirto historiaan numeroina ja sanoina + sijoitetaan se "komentotunnistettu"-muuttujaan, jolloin displaytask
	//reagoi samalla tavalla kuin liikkeest� tunnistettuun liikkeeseen.
	if(uudetsiirrot<0){
		System_printf("ei pysty");
		System_flush();
	}else{


	if(komentohistorianrot[uudetsiirrot]==1){
		komentotunnistettu=2;
		//uudetsiirrot--;
		sprintf(komento, "DOWN");
		komentohistorianrot[siirrot] = 2;
		komentohistoria[siirrot] = "Alas";
		siirrot++;
	}
	else if(komentohistorianrot[uudetsiirrot]==2){
		komentotunnistettu=1;
		//uudetsiirrot--;
		sprintf(komento, "UP");
		komentohistorianrot[siirrot] = 1;
		komentohistoria[siirrot] = "Ylos";
		siirrot++;
	}
	else if(komentohistorianrot[uudetsiirrot]==3){
		komentotunnistettu=4;
		//uudetsiirrot--;
		sprintf(komento, "LEFT");
		komentohistorianrot[siirrot] = 4;
		komentohistoria[siirrot] = "Vasen";
		siirrot++;
	}
	else if(komentohistorianrot[uudetsiirrot]==4){
		komentotunnistettu=3;
		//uudetsiirrot--;
		sprintf(komento, "RIGHT");
		komentohistorianrot[siirrot] = 3;
		komentohistoria[siirrot] = "Oikea";
		siirrot++;
	}
	sprintf(sanaa,"%i",uudetsiirrot);
	System_printf(sanaa);
	}
}

//"Ylemm�n" napin k�sittelij�
void buttonFxn(PIN_Handle handle, PIN_Id pinId) {
	noice = 1; //vieritksen ��ni halutaan

	//Jos ollaan menussa, laitetaan kursoria eteenp�in
	if(tila == 5){
		//System_printf("nappi");
		//System_flush();
		kursori++;
		if(kursori==3){
			kursori=0;
		}
	}

	//Jos peli menossa, siirryt��n satunnaiseen suuntaan
	if(tila == 1){
		random();
		//System_printf("%i\n",peliaika);
		//System_printf("%i\n",siirrot);
		//System_flush();
		//for(k=0;k<siirrot;k++){
		//	System_printf("%s\n", komentohistoria[k]);
		//	System_flush();
		//}
	}

	//Kun menn��n kattoon edellisen pelin siirtoja, palataan t�ll� napilla menuun
	if(tila == 8){
		tila = 6;
	}
}

void buttonFxn2(PIN_Handle handle2, PIN_Id pinId2){
	noice = 2; //valinnan ��ni halutaan
	//Menussa kutsutaan funktiota joka tulkistee, ett� pit� pit�� tapahtua seuraavaksi
	if(tila == 5){
		valitsemisfunktio();
	}

	//Pelin ollessa k�ynniss� peruutus:
	if(tila==1 || tila==3){
		tila = 3;
		uudetsiirrot--;

		halutaanedellinen = 1;
		edellinen2();
		//kelaus++;
	}

	//Edellist� peli� tutkittaessa vieritet��n listaa, jos ei mahdu kerralla ruudulle
	if(tila==8){
		if(siirrot > 11){
			vieritys++;
		}
		tila=7;
	}
}


void random(){
	tila = 3;
	srand(time(NULL));
	int rando = rand() % 4 + 1;
	System_printf("%i", rando);
	System_flush();
	komentotunnistettu = rando;
	if(rando==1){
		sprintf(komento, "UP");
		komentohistorianrot[siirrot] = 1;
		komentohistoria[siirrot] = "Ylos";
		siirrot++;
	}
	else if(rando==2){
		sprintf(komento, "DOWN");
		komentohistorianrot[siirrot] = 2;
		komentohistoria[siirrot] = "Alas";
		siirrot++;
	}
	else if(rando==3){
		sprintf(komento, "RIGHT");
		komentohistorianrot[siirrot] = 3;
		komentohistoria[siirrot] = "Oikea";
		siirrot++;
	}
	else if(rando==4){
		sprintf(komento, "LEFT");
		komentohistorianrot[siirrot] = 4;
		komentohistoria[siirrot] = "Vasen";
		siirrot++;
	}
	uudetsiirrot = siirrot;
}

//Kursin sijainnin perusteella asetetaan oikeat tilat p��lle.
void valitsemisfunktio(){
	if(kursori == 0){
		pelimenossa = 1;
		System_printf("aloitetaan");
		System_flush();
		alotusaika = Clock_getTicks();
		tila = 6;
	}
	if(kursori == 1){
		tila = 7;
	}
	if(kursori == 2){
		resettifunktio(); //Funktio jolla "resetoidaan" muisti (ts. nollataan kaikki historiat)
	}
	//if(kursori == 3){ //testi
	//	tila = 11;
	//}
	//if(kursori == 4){ //testi
	//	tila = 12;
	//}
}

//T�ll� funktiolla vain yksinkertaisesti nollataan kaikki muistissa oleva:
void resettifunktio(){
	//System_printf("T�ss� reset�id��n ehk�");
	//System_flush();
	int i;
	for(i=0;i<siirrot;i++){
		komentohistoria[i] = "";
		komentohistorianrot[i] = 0;
	}
	siirrot = 0;
	uudetsiirrot = 0;
	vieritys=0;
}


//Kommunikointi taski:
char message[32];
char rmessage[80];
char lmessage[80] = "55,LOST GAME";
char wmessage[80] = "55,WIN";
uint16_t senderAddr;
Void commTaskFxn(UArg arg0, UArg arg1) {

    // Radio vastaanottotilaan
	int32_t result = StartReceive6LoWPAN();
	if(result != true) {
		System_abort("Wireless receive mode failed");
	}

    while (1) {

        // Jos tulee viesti, ja se on meille tarkoitettu, laitetaan tilat oikein sen perusteella:
    	if (GetRXFlag() == true) {
    		memset(rmessage,0,80);
    		Receive6LoWPAN(&senderAddr, rmessage, 80);
    		//System_printf(rmessage);
    		//System_flush();

    		if(strcmp(wmessage, rmessage) == 0){
    			//System_printf("voitto tuli");
    			//System_flush();
    			tila=11;
    		}
    		else if(strcmp(lmessage, rmessage) == 0){
    			//System_printf("Häviö");
    			//System_flush();
    			tila=12;
    		}
        }

    	//Jos tila 10, l�hetet��n eventti ja palataan perus tilaan 1 (ker��m��n dataa)
    	if(tila==10){
    		sprintf(message,"event:%s",komento);
    		if(komentotunnistettu != 0){
    			if(halutaanedellinen==1){
    				halutaanedellinen=0;
    				komentotunnistettu=0;
    			}
    			Send6LoWPAN(IEEE80154_SERVER_ADDR, message, strlen(message)); //MIK� OSOTE?
    			StartReceive6LoWPAN();
    			System_printf(message);
    			System_flush();
    		}
    		tila=1;
    		}
    	}
    	// Absolutely NO Task_sleep in this task!!
}




Int main(void) {

    // Taskien muuttujat
	Task_Handle sensorTask;
	Task_Params sensorTaskParams;

	Task_Handle displayTask;
	Task_Params displayTaskParams;

	Task_Handle commTask;
	Task_Params commTaskParams;


    // Initialize board
    Board_initGeneral();
    Board_initI2C();

    hMpuPin = PIN_open(&MpuPinState, MpuPinConfig);
    if (hMpuPin == NULL) {
    	System_abort("Pin open failed!");
    }

    //Sensor task:
    Task_Params_init(&sensorTaskParams);
    sensorTaskParams.stackSize = STACKSIZE;
    sensorTaskParams.stack = &sensorTaskStack;
    sensorTaskParams.priority=2;
    sensorTask = Task_create(sensorTaskFxn, &sensorTaskParams, NULL);
    if (sensorTask == NULL) {
      	System_abort("Task create failed!");
    }

    //Display task:
    Task_Params_init(&displayTaskParams);
    displayTaskParams.stackSize = STACKSIZE;
    displayTaskParams.stack = &displayTaskStack;
    displayTaskParams.priority=2;
    displayTask = Task_create(displayTaskFxn, &displayTaskParams, NULL);
    if (displayTask == NULL) {
       	System_abort("Task create failed!");
    }


    //Nappien k�sittelyhommat:
    buttonHandle = PIN_open(&buttonState, buttonConfig);
        if(!buttonHandle) {
           System_abort("Error initializing button pins\n");
    }

    if (PIN_registerIntCb(buttonHandle, &buttonFxn) != 0) {
       System_abort("Error registering button callback function");
    }

    buttonHandle2 = PIN_open(&buttonState2, buttonConfig2);
        if(!buttonHandle2) {
           System_abort("Error initializing button pins\n");
    }

    if (PIN_registerIntCb(buttonHandle2, &buttonFxn2) != 0) {
       System_abort("Error registering button callback function");
    }

    //Ledien handlet:
    ledHandle = PIN_open(&ledState, ledConfig);
    led2Handle = PIN_open(&led2State, led2Config);


    //Kommunikaatio task:

    Init6LoWPAN(); // This function call before use!

    Task_Params_init(&commTaskParams);
    commTaskParams.stackSize = STACKSIZE;
    commTaskParams.stack = &commTaskStack;
    commTaskParams.priority=1;

    commTask = Task_create(commTaskFxn, &commTaskParams, NULL);
    if (commTask == NULL) {
    	System_abort("Task create failed!");
    }

    //Buzzeri k�ytt��n:
    BuzzerHandle = PIN_open(&BuzzerState, Buzzer);

    if (!BuzzerHandle) {
        System_abort("Buzzer failed!");
    }

    buzzerOpen(BuzzerHandle);
    /* Sanity check */
    System_printf("Hello world!\n");
    System_flush();
    
    /* Start BIOS */
    BIOS_start();

    return (0);
}



//Vanhat analysointi ja peruutusfunktiot. Molemmissa n�iss� oli jotain vikaa, mink� takia uudet tehty.


////int j;
//char laskut1[100] = "";
//char laskut2[100] = "";
////char komento[10] = "";
//float max_ax;
//float min_ax;
//float max_ay;
//float min_ay;
//float ka_ax;
//float ka_ay;
//float sum_ax;
//float sum_ay;
//int min_ay_index;
//int max_ay_index;
//int min_ax_index;
//int max_ax_index;
//int j;
//void analysointi(){
//	sum_ax = 0;
//	sum_ay = 0;
//	for(j=0;j<10;j++){
//		sum_ax = sum_ax + ax[j];
//		sum_ay = sum_ay + ay[j];
//	}
//	ka_ax = sum_ax / 8;
//	ka_ay = sum_ay / 8;
//
//	max_ax = ax[0];
//	max_ay = ay[0];
//	for(j=0;j<10;j++){
//		if(ax[j]>max_ax){
//			max_ax = ax[j];
//			max_ax_index = index[j];
//		}
//		if(ay[j]>max_ay){
//			max_ay = ay[j];
//			max_ay_index = index[j];
//		}
//	}
//
//	min_ax = ax[0];
//	min_ay = ay[0];
//	for(j=0;j<10;j++){
//		if(ax[j]<min_ax){
//			min_ax = ax[j];
//			min_ax_index = index[j];
//		}
//		if(ay[j]<min_ay){
//			min_ay = ay[j];
//			min_ay_index = index[j];
//		}
//	}
//	//sprintf(laskut1,"sum_ax=%f, sum_ay=%f, ka_ax=%f, ka_ay=%f\n",sum_ax, sum_ay, ka_ax, ka_ay);
//	//sprintf(laskut2, "maxax=%f, max_ay=%f, min_ax=%f, min_ay=%f\n",max_ax, max_ay, min_ax, min_ay);
//	//System_printf(laskut1);
//	//System_flush();
//	//System_printf(laskut2);
//	//System_flush();
//
//	if(max_ay_index < min_ay_index && (ka_ay>ka_ax || ka_ay<ka_ax)){
//		System_printf("RIGHT");
//		System_flush();
//	}
//	if(min_ay_index < max_ay_index && (ka_ay>ka_ax || ka_ay<ka_ax)){
//		System_printf("LEFT");
//		System_flush();
//	}
//	if(max_ax_index > min_ax_index && (ka_ay<ka_ax || ka_ay>ka_ax)){
//		System_printf("DOWN");
//		System_flush();
//	}
//	if(min_ax_index > max_ax_index && (ka_ay<ka_ax || ka_ay>ka_ax)){
//		System_printf("UP");
//		System_flush();
//	}
//}

//int haluttu;
//int sallihistoria;
//void edellinen(){
//	System_printf("edellinen pit�s tulla t�ss�");
//	System_flush();
//
//	haluttu = uudetsiirrot - kelaus + 1;
//	if(haluttu <= 0){
//		haluttu = 0;
//		System_printf("Ei pysty");
//		sallihistoria = 0;
//		System_flush();
//	}else{sallihistoria = 1;}
//
//
//	if(sallihistoria == 1){
//		System_printf("historia sallittiin \n");
//		System_flush();
//		if(komentohistorianrot[haluttu] == 1){
//			komentotunnistettu = 1;
//			if(siirrot < 12) {
//				komentohistoria[siirrot] = "Ylos";
//				komentohistorianrot[siirrot] = 1;
//			}
//			siirrot++;
//			//kelaus++;
//			System_printf("edellinenylos");
//			System_flush();
//		}
//
//		if(komentohistorianrot[haluttu] == 2){
//			komentotunnistettu = 2;
//			if(siirrot < 12) {
//				komentohistoria[siirrot] = "Alas";
//				komentohistorianrot[siirrot] = 2;
//			}
//			siirrot++;
//			//kelaus++;
//			System_printf("edellinenalas");
//			System_flush();
//		}
//
//		if(komentohistorianrot[haluttu] == 3){
//			komentotunnistettu = 3;
//			if(siirrot < 12) {
//				komentohistoria[siirrot] = "Oikealle";
//				komentohistorianrot[siirrot] = 3;
//			}
//			siirrot++;
//			//kelaus++;
//			System_printf("edellinenoikea");
//			System_flush();
//		}
//
//		if(komentohistorianrot[haluttu] == 4){
//			komentotunnistettu = 4;
//			if(siirrot < 12) {
//				komentohistoria[siirrot] = "Vasemmalle";
//				komentohistorianrot[siirrot] = 4;
//			}
//			siirrot++;
//			//kelaus++;
//			System_printf("edellinenvasen");
//			System_flush();
//		}
//
//	}
//	if(sallihistoria == 0){
//		komentotunnistettu = 0;
//	}
//	//sprintf(vittu, "%i", kelaus);
//	//System_printf(vittu);
//	//System_flush();
//
//	//halutaanedellinen = 0;
//}
