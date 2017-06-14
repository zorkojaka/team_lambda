

#include <maze_navigator/logic.h>



//GLOBALNE SPREMENLJIVKE
int poznamoring = 0;
int poznamocilinder = 0;
int zaporednimoski = 1;
int inforingspol = 0;
int infocilinderspol = 0;
person *manhead;
person *womanhead;

//Struct person
struct person
{
	int id;
	int spol;
	int starost;
	int visina;

	int inforing; //1 ima -1 nima 0 nedefiniran
	int infocilinder;
	int truth;		//1 resnica -1 laž 0 nedefiniran
	int prisotnost; // 0 nedefiniran 1 prisoten -1 neprisoten
	person *next;
};

//ustvarimo novo osebo in jo dodamo na pravi seznam glede na spol
//manhead ali womanhead.
void addperson(int id, int spol, int starost, int visina)
{
	person *kaz;
	person *nov;

	if (spol == 1)
	{				   //moski
		kaz = manhead; //zacetk moskega lista
		while (kaz->next != NULL)
		{
			kaz = kaz->next;
		}
		//dobimo da kaz kaže na zadnje el.

		kaz->next = new person;
		kaz = kaz->next;

		//VNOS PODATKOV
		kaz->id = id;
		kaz->spol = 1;
		kaz->starost = starost;
		kaz->visina = visina;
		kaz->inforing = 0;
		kaz->infocilinder = 0;
		kaz->truth = 1;
		kaz->prisotnost = 0;
		kaz->next = NULL;
	}
	else
	{ //zenska
		kaz = womanhead;
		while (kaz->next != NULL)
		{
			kaz = kaz->next;
		}
		//dobimo da kaz kaže na zadnje el.

		kaz->next = new person;
		kaz = kaz->next;

		//VNOS PODATKOV
		kaz->id = id;
		kaz->spol = 2;
		kaz->starost = starost;
		kaz->visina = visina;
		kaz->inforing = 0;
		kaz->infocilinder = 0;
		kaz->truth = 0;
		kaz->prisotnost = 0;
		kaz->next = NULL;
	}
}

//incializacija vseh oseb v lista man in woman
void spoznavanjeoseb()
{

	//incializacija glav listov
	manhead = new person;
	manhead->id = 0;
	manhead->spol = 1;
	manhead->next = NULL;

	womanhead = new person;
	womanhead->id = 0;
	womanhead->spol = 2;
	womanhead->next = NULL;

	//DODAMO VSE OSEBE (id,spol(1=man,2=woman),starost,visina,inforing,infocilinder,truth; (1=true,-1=false,0=undefined))

	//MAN DODAMO JIH NA LIST manhead->Harry
	//1. Harry
	addperson(1, 1, 18, 165);
	//2. Peter
	addperson(2, 1, 25, 182);
	//3. Elvis
	addperson(3, 1, 34, 181);
	//4. Forrest
	addperson(4, 1, 42, 183);
	//5. Tesla
	addperson(5, 1, 45, 188);
	//6. Albert
	addperson(6, 1, 71, 180);

	// WOMAN  DODAMO JIH NA NOV LIST womanhead->Ilka...
	//7. Ilka
	addperson(7, 2, 26, 172);
	//8. Adele
	addperson(8, 2, 29, 175);
	//9. Scarlet
	addperson(9, 2, 31, 160);
	//10. Lindsay
	addperson(10, 2, 32, 178);
	//11. Tina
	addperson(11, 2, 33, 171);
	//12. Ellen
	addperson(12, 2, 59, 170);
}

//funkcija vrne ime osebe z podanim ID-jem
string returnName(int id)
{
	switch (id)
	{
	case 1:
		return "Harry";
	case 2:
		return "Peter";
	case 3:
		return "Elvis";
	case 4:
		return "Forrest";
	case 5:
		return "Tesla";
	case 6:
		return "Albert";
	case 7:
		return "Ilka";
	case 8:
		return "Adele";
	case 9:
		return "Scarlett";
	case 10:
		return "Lindsey";
	case 11:
		return "Tina";
	case 12:
		return "Ellen";
	default:
		return "Nevem";
	}
}

//Print podanega linked lista, podamo manhead/womanhead napiše vse podatke
void printlist(person *head)
{
	person *kaz = head;
	while (kaz->next != NULL)
	{
		kaz = kaz->next;
		printf("Oseba:%d, spol: %d, starost: %d, visina;%d, inforing: %d, infocilinder: %d, truth: %d, prisotnost: %d\n", kaz->id, kaz->spol, kaz->starost, kaz->visina, kaz->inforing, kaz->infocilinder, kaz->truth, kaz->prisotnost);
		fflush(stdout);
	}
}

// preveri če že vemo kdo ve kje je ring vrne ID, če ne vemo vrne 0
//nastavi poznamoring in poznamocilinder na ID če poznamo, drugače 0
int inforingcheck(person *head)
{
	person *kaz = head->next;
	int enka = 0;
	int stnicel = 0;
	int niclaid = 0;
	while (kaz != NULL)
	{

		//gledamo če kdo ve
		if (kaz->inforing == 1)
		{
			enka = kaz->id;
			poznamoring = enka;
			return enka;
		}
		//gledamo kok mamo še neznanih
		if (kaz->prisotnost > -1 && kaz->inforing == 0)
		{
			stnicel++;
			niclaid = kaz->id;
		}
		kaz = kaz->next;
	}
	if (stnicel == 1)
	{
		kaz = head->next;
		while (kaz->id != niclaid && kaz != NULL)
		{
			kaz = kaz->next;
		}

		kaz->inforing = 1;
		poznamoring = niclaid;

		return niclaid;
	}
	return 0;
}

// preveri če že vemo kdo ve kje je cilinder
int infocilindercheck(person *head)
{
	person *kaz = head->next;
	int enka = 0;
	int stnicel = 0;
	int niclaid = 0;

	//gremo čez vse
	while (kaz != NULL)
	{
		//gledamo če kdo ve
		if (kaz->inforing == 1)
		{
			enka = kaz->id;
			poznamocilinder = enka;
			return enka;
		}
		//gledamo kok mamo še neznanih
		if (kaz->prisotnost > -1 && kaz->infocilinder == 0)
		{
			stnicel++;
			niclaid = kaz->id;
		}
		//premaknemo naprej
		kaz = kaz->next;
	}

	//če je samo še en prisoten in nezana pol on ve
	if (stnicel == 1)
	{
		kaz = head->next;
		//pokazemo nanga
		while (kaz->id != niclaid && kaz != NULL)
		{
			kaz = kaz->next;
		}

		kaz->infocilinder = 1;
		poznamocilinder = niclaid;

		return niclaid;
	}
	//še ne vemo
	return 0;
}

/*POGOVOR:
- z žensko: vprašamo če je ženska -> rensica/laž
- 1. moški: vprašamo če ima info kje je ring/cilinder moški
- 2. moški: za ring preverja po imenu 
- nadaljni moški: če vemo za ring preverjamo za cilinder*/
void pogovor(struct person per)
{
	struct person *kaz;
	int odg = 1;

	stringstream stream;

	//pogovor ženske
	if (per.spol == 2)
	{
		system("rosrun sound_play say.py 'Hi. Are you a woman'");
		kaz = womanhead;
		if (odg == 1)
		{
			//POMENI DA GOVORI RESNICO
			while (kaz->id != per.id)
			{
				kaz = kaz->next;
			}
			kaz->truth = 1;
			kaz->prisotnost = 1;
		}
		else
		{ //LAŽE
			while (kaz->id != per.id)
			{
				kaz = kaz->next;
			}
			kaz->truth = -1;
			kaz->prisotnost = 1;
		}
		//KONEC POGOVORA Z ŽENSKO
	}
	else if (per.spol == 1)
	{
		//POGOVOR Z MOŠKIM
		kaz = manhead->next;
		//kažemo na pravega
		while (kaz->id != per.id)
		{
			kaz = kaz->next;
		}
		kaz->prisotnost = 1;

		//pogovor s prvim moškim
		if (zaporednimoski == 1)
		{
			zaporednimoski++;
			system("rosrun sound_play say.py 'Is the person who knows which ring is magical a man?'");
			//odg="yes";//yes
			if (odg == 1)
			{
				//woman cant have info about magic rings
				kaz = womanhead->next;
				inforingspol = 1;
				while (kaz != NULL)
				{
					kaz->inforing = -1;
					kaz = kaz->next;
				}
			}
			else
			{
				//man cant have info about magic rings
				kaz = manhead->next;
				inforingspol = 2;
				while (kaz != NULL)
				{
					kaz->inforing = -1;
					kaz = kaz->next;
				}
			}
			//2. VPRAŠANJE
			system("rosrun sound_play say.py 'Is the person who knows which location is right a man?'");
			if (odg == 1)
			{
				//woman cant have info about location
				kaz = womanhead->next;
				infocilinderspol = 1;
				while (kaz != NULL)
				{
					kaz->infocilinder = -1;
					kaz = kaz->next;
				}
			}
			else
			{
				//man cant have info about magic rings
				kaz = manhead->next;
				infocilinderspol = 2;
				while (kaz != NULL)
				{
					kaz->infocilinder = -1;
					kaz = kaz->next;
				}
			}
		}
		//naprej je za drugega in tretjega moškega
		else if (zaporednimoski == 2)
		{
			zaporednimoski++;
			//drugi moski ugotovimo kdo ve za ring
			if (inforingspol == 1)
			{
				kaz = manhead;
			}
			else
			{
				kaz = womanhead;
			}
			//kaz po zanki kaže na un spol ki to ve.
			while (kaz->inforing != 0 && kaz != NULL && kaz->prisotnost > -1)
			{
				kaz = kaz->next;
			} //damo na prvega, za kerga ne vemo a ve al ne.

			//1.VPRAŠANJE

			stream << "rosrun sound_play say.py "
				   << "'does" << returnName(per.id) << "know where is ring?'";
			system(stream.str().c_str());
			if (odg == 1)
			{
				kaz->inforing = 1;
			}
			else
			{
				kaz->inforing = -1;
			}

			//2. VPRAŠANJE
			while (kaz->inforing != 0 && kaz != NULL && kaz->prisotnost > -1)
			{
				kaz = kaz->next;
			}
			stream << "rosrun sound_play say.py "
				   << "'does" << returnName(per.id) << "know where is ring?'";
			system(stream.str().c_str());

			if (odg == 1)
			{
				kaz->inforing = 1;
			}
			else
			{
				kaz->inforing = -1;
			}
		}
		else if (zaporednimoski > 2)
		{
			zaporednimoski++;
			//nastavmo kazalec na pravi spol

			if (poznamoring == 0)
			{ //če še ne poznamo kdo ve kje je ring
				if (inforingspol == 1)
				{
					kaz = manhead;
				}
				else
				{
					kaz = womanhead;
				}

				while (kaz->inforing != 0 && kaz != NULL && kaz->prisotnost > -1)
				{
					kaz = kaz->next;
				}
				if (kaz != NULL)
				{
					//1.VPRAŠANJE
					stream << "rosrun sound_play say.py "
						   << "'does" << returnName(per.id) << "know where is ring?'";
					system(stream.str().c_str());
					if (odg == 1)
					{
						kaz->inforing = 1;
					}
					else
					{
						kaz->inforing = -1;
					}
				}

				//2.VPRAŠANJE
				while (kaz->inforing != 0 && kaz != NULL && kaz->prisotnost > -1)
				{
					kaz = kaz->next;
				}
				if (kaz != NULL)
				{
					stream << "rosrun sound_play say.py "
						   << "'does" << returnName(per.id) << "know where is ring?'";
					system(stream.str().c_str());
					if (odg == 1)
					{
						kaz->inforing = 1;
					}
					else
					{
						kaz->inforing = -1;
					}
				}

				//ČE SMO RING ŽE POGRUNTAL
			}
			else if (poznamocilinder == 0)
			{
				//nastavmo kazalec na pravi spol
				if (infocilinderspol == 1)
				{
					kaz = manhead;
				}
				else
				{
					kaz = womanhead;
				}
			}

			//1. VPRAŠANJE
			while (kaz->inforing != 0 && kaz != NULL && kaz->prisotnost > -1)
			{
				kaz = kaz->next;
			}

			if (kaz != NULL)
			{
				stream << "rosrun sound_play say.py "
					   << "'does" << returnName(per.id) << "know where is ring?'";
				system(stream.str().c_str());
				if (odg == 1)
				{
					kaz->infocilinder = 1;
				}
				else
				{
					kaz->infocilinder = -1;
				}
			}
			//2.vprašanje
			while (kaz->inforing != 0 && kaz != NULL && kaz->prisotnost > -1)
			{
				kaz = kaz->next;
			}

			if (kaz != NULL)
			{
				//TREBA SESTAVT STRING UKAZ + IME +...

				stream << "rosrun sound_play say.py "
					   << "'does" << returnName(per.id) << "know where is ring?'";
				system(stream.str().c_str());
				if (odg == 1)
				{
					kaz->infocilinder = 1;
				}
				else
				{
					kaz->infocilinder = -1;
				}
			}
			else
			{
				person *ringhead;
				person *cilinderhead;
				if (inforingspol == 1)
				{
					ringhead = manhead;
				}
				else
				{
					ringhead = womanhead;
				}

				if (infocilinderspol == 1)
				{
					cilinderhead = manhead;
				}
				else
				{
					cilinderhead = womanhead;
				}
				printf("VEMO KDO VE KEJ, Ring:%d, Cilinder:%d\n", inforingcheck(ringhead), infocilindercheck(cilinderhead));
			}
		}
	}
}

/*
MAIN

spoznavanjeoseb();
//manhead->Harry->Peter->Elvis->Forrest->Tesla->Albert
//womanhead->Ilka->Adele->Scarlet->Lindsay->Tina->Ellen	

struct person *kaz;
int id=getid(); //from stream 
kaz= //nastavmo na osebo s pravim id, ki ga prepoznamo
pogovor(*kaz);	
...
int ringID = inforingcheck(manhead);
int cilinderID = inforingcheck(womanhead);


*/
