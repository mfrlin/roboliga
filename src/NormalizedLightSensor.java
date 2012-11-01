/*
NormalizedLightSensor je podrazred LightSensor, ki implementira tudi dinamicno normaliziranje 
meritev - vrne vrednost med 0 in 100.
Kaj pomeni dinamicno? Ni treba predhodno izmeriti maximalni in minimalne signale (crno barvo
traku in belo podlage), vendar sam prilagaja interval na katerega normalizira na merive.

Uporaba:

	=========================
	Z normalizacijo: 
			sensor1 = new NormalizedLightSensor(SensorPort.S1);
			sensor2 = new NormalizedLightSensor(SensorPort.S2);
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
	private int boundingMax = 50;  /* Maksimalna normalizirana vrednost meritev */
	private int boundingMin = 50; /* Minimalna normalizirana vrednost meritev */

	private int activateAfterCount = 10; /* Stevilo meritev preden zacne normalizirati */

	/*
	* @params:
	* 	LightSensor sensor: ime svetlobnega senzorja
	*/
	public NormalizedLightSensor(SensorPort sensorPort){
		super(sensorPort);
	}

	/*
	* @params:
	* 	LightSensor sensor: ime svetlobnega senzorja
	* 	boolean normalize: ce je normalize=false, potem ne bo normaliziral vrednosti,
	* 		in bo vrnil kar raw prebrane podatke iz senzorja.
	*/
	public NormalizedLightSensor(SensorPort sensorPort, boolean normalize) {
		super(sensorPort);
		this.normalize = normalize;
	}

	/*
	* Podobno kot LightSensor.getLightValue(), le da po potrebi normalizira podatke
	* @params: none
	*	@returns: normalizirano (ali ne) vrednost, ki jo prebere iz senzorjev
	*/
	private int getLocallyNormalizedValue() {
		int rawValue = this.getLightValue();

		if (normalize == false) {
			return rawValue;
		} else {
			int normalizedValue = normalizeValue(rawValue);
			return normalizedValue;
		}
	}

	/* 
	* Vrne verndost, ki je normalizirana (lokalno), a primerljiva z metrivam drugega senzorja.
	* Normlizira normalizirano vrednost na interval [0..100]
	* @returns:
	* 	int value: normalizirana in primerljiva vrednost z drugimi svetlobniki senzorji.
	*/
	public int getValue() {
		int locallyNormalizedValue = getLocallyNormalizedValue();
		return (int)((100.0 * locallyNormalizedValue) / (boundingMax - boundingMin));
	}

	/*
	* Algoritem za normaliziranje prebranih vrednosti iz senzorja
	* Opis algoritma:
	* 	Vhodne podatke skalira na interval [boundingMin, boundingMax]
	* Algoritem se aktivira sele potem, ko je prebral vsaj 10 meritev iz senzorjev.
	* Predpostavlja namrec, da bo v teh 10 meritvah dobil vsaj 1 iz crne, in vsaj 1 iz bele barve
	* @params: 
	*		int rawValue: prebrana vrednost iz senzorja
	*/
	private int normalizeValue(int rawValue) {

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

		/* Normaliziraj meritve */
		if (rawValue < boundingMin) {
			return boundingMin;
		} else if (rawValue > boundingMax) {
			return boundingMax;
		} else {
			return (int) (0.01 * (boundingMax - boundingMin) * rawValue + boundingMin); /* krizni racun */
 		}
	}
}