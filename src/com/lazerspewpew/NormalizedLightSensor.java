package com.lazerspewpew;
/*
NormalizedLightSensor je podrazred LightSensor, ki implementira tudi dinamicno in staticno normaliziranje 
meritev - vrne vrednost med 0 in 100.
Kaj pomeni dinamicno? Ni treba predhodno izmeriti maximalni in minimalne signale (crno barvo
traku in belo podlage), vendar sam prilagaja interval na katerega normalizira na merive.
Kaj pomeni staticno? Vnaprej se dolocimo meje, ki predstavljajo crno in belo barvo. Meja se ne spreminja

Vsak senzor ima svojo mejo. Tako lahko dosezemo dobro delovanju, tudi ce kateri senzor vraca precej
razlicne vrednosti.

Uporaba:

	=========================
	Z dinamicno normalizacijo: 
			sensor1 = new NormalizedLightSensor(SensorPort.S1);
			sensor2 = new NormalizedLightSensor(SensorPort.S2);
			int reading1 = sensor1.getValue();
			int reading2 = sensor2.getValue();

			Nato lahko primerjamo meritve:
			int difference = reading2 - reading1;
	========================
	S staticno normalizacijo
		sensor1 = new NormalizedLightSensor(SensorPort.S1, 20, 60);  // port, min, max 
		sensor2 = new NormalizedLightSensor(SensorPort.S2, 20, 60);
		int reading1 = sensor1.getValue();
		int reading2 = sensor2.getValue();

		Nato lahko primerjamo meritve:
		int difference = reading2 - reading1;

	========================
	Brez normalizacije:
		popolnoma enako kot zgoraj, le da pri construktorju nastavimo drugi parameter na false.
		sensor = new NormalizedLightSensor(SensorPort.S1, false);

*/

import lejos.nxt.LightSensor;
import lejos.nxt.SensorPort;

public class NormalizedLightSensor extends LightSensor {

	private boolean normalize = true;
	private boolean normalizeToFixedBoundaries = false; /* Ce vnaprej nastavimo meje za min in max senzor value */
	
	private int boundingMax = 50;  /* Maksimalna normalizirana vrednost meritev */
	private int boundingMin = 50; /* Minimalna normalizirana vrednost meritev */

	private int activateAfterCount = 10; /* Stevilo meritev preden zacne normalizirati */

	/*
	* Uporabljen za dinamicno normalizacijo
	* @params:
	* 	SensorPort sensorPort: port svetlobnega senzorja
	*/
	public NormalizedLightSensor(SensorPort sensorPort){
		super(sensorPort);
	}

	/*
	* Uporabljen, ce ne zelimo normalizacije
	* @params:
	* 	SensorPort sensorPort: port svetlobnega senzorja
	* 	boolean normalize: ce je normalize=false, potem ne bo normaliziral vrednosti,
	* 		in bo vrnil kar raw prebrane podatke iz senzorja.
	*/
	public NormalizedLightSensor(SensorPort sensorPort, boolean normalize) {
		super(sensorPort);
		this.normalize = normalize;
	}

	/*
	* Konstruktor, ki se uporablja ce zelimo normalizirati na predefiniran interval, torej 
	* ce poznamo meritev senzorja na beli in crni podlagi.
	* @params:
	* 	SensorPort sensorPort: port svetlobnega senzorja
	* 	int boundingMin: spodnja meja za normalizacijo (meritev bele barve)
	* 	int boundingMax: zgornja meja za normalizacijo (meritev crne barve)
	*/
	public NormalizedLightSensor(SensorPort sensorPort, int boundingMin, int boundingMax){
		super(sensorPort);
		this.normalizeToFixedBoundaries = true;
		this.boundingMin = boundingMin;
		this.boundingMax = boundingMax;
	}

	/*
	Nastavi normalizacijo na staticno, na podane meje.
	*/
	public void setFixedBoundaries(int boundingMin, int boundingMax){
		this.normalizeToFixedBoundaries = true;
		this.boundingMin = boundingMin;
		this.boundingMax = boundingMax;
	}

	/* 
	* Vrne verndost, ki je normalizirana (lokalno), a primerljiva z metrivam drugega senzorja.
	* Normlizira normalizirano vrednost na interval, ki je lahko dinamicen ali staticen 
	* (odvisno od  uporabljenega konstruktorja)
	* @returns:
	* 	int value: normalizirana in primerljiva vrednost z drugimi svetlobniki senzorji.
	*/
	public int getValue() {

		int rawValue = this.getLightValue();

		if (normalize == false) {
			return rawValue;
		} else if (normalizeToFixedBoundaries == true) {
			return normalizeValueToStaticBounds(rawValue);
		} else {
			return normalizeValueToDynamicBounds(rawValue);
		}
	}

	/*
	* Algoritem za staticno normaliziranje prebranih vrednosti iz senzorja,
	* ce smo vnaprej nastavili boundingMin in boundingMax na minimalno in maksimalno
	* vrednost, ki jo bere senzor na danih svetlobnih pogojih
	*
	* @params:
	* 	int rawValue: prebrana vrednost iz senzorja
	* @returns
	* 	int value: normalizirano vrednost na intrevalu [0..100]
	*/
	private int normalizeValueToStaticBounds(int rawValue) {

		if (rawValue <= boundingMin) {
			return 0;
		} else if (rawValue >= boundingMax) {
			return 100;
		} else {
			return (int)(100.0 * ( rawValue - boundingMin ) / ( boundingMax - boundingMin ));
 		}
	}

	/*
	* Algoritem za dinamicno normaliziranje prebranih vrednosti iz senzorja
	* Opis algoritma:
	* 	Vhodne podatke skalira na interval [boundingMin, boundingMax]
	* Algoritem se aktivira sele potem, ko je prebral vsaj 10 meritev iz senzorjev.
	* Predpostavlja namrec, da bo v teh 10 meritvah dobil vsaj 1 iz crne, in vsaj 1 iz bele barve
	* @params: 
	*		int rawValue: prebrana vrednost iz senzorja
	*/
	private int normalizeValueToDynamicBounds(int rawValue) {

		/* Upostevaj meritve za dinamicno postavitev meje normalizacije */
		if (rawValue < boundingMin) {
			/* Da preprecim odvecem sum, uposteval samo del razlike. Scasoma konvergira. */
			boundingMin -= (boundingMin - rawValue) * 0.8; /* == 0.2* boundingMin + 0.8 * rawValue */
		}
		if (rawValue > boundingMax) {
			boundingMax += (rawValue - boundingMax) * 0.8; /* == 0.2* boundingMin + 0.8 * rawValue */
		}

		/* Ce nima dovolj meritev, vrni kar prave meritve. */
		if (activateAfterCount > 0) {
			activateAfterCount--;
			return rawValue;
		}

		/* Normaliziraj meritve na nove meje */
		return normalizeValueToStaticBounds(rawValue);
	}
}