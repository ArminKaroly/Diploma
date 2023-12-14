# Károly Ármin diplomamunka
<sub><sup>THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY WARRANTY.  IT IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. THE ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE SOFTWARE IS WITH YOU. SHOULD THE SOFTWARE PROVE DEFECTIVE, YOU ASSUME THE COST OF ALL NECESSARY SERVICING, REPAIR OR CORRECTION. </sup></sub>

## Előkészületek
- A program működéséhez szükséges könyvtárakat a *requirements.txt*  file tartalmazza, ezeknek az installálásához a *pip3 install < könyvtár_név >*  terminal kódot javaslom

- A programkódok és  a futtatáshoz szükséges egyéb file-ok  a *scripts* mappában találhatóak

## Program futtatása
Az applikációs program futtatásához a *main.py*  script futtatása szükséges (szükséges hogy többi script is a *main.py*  mellett helyezkedjen el)

## A kalibrációs folyamat elvégzése
#### -- <ins>Inicializáció:</ins>
- A kalibrációs applikációt futtató *main.py* elindítását követően szükséges a bemeneti adatok megadása a felugró ablak bal oldalán. 
- Először is szükséges a robotkar hálózati IP címének a megadása. Azon esetben, hogyha ez elérhető, és megfelelő, akkor a "Connect to Socket" felirat mellett egy fehér pipa jelenik meg. 
- Ezt követően a megfelelő kamera kiválasztása szükséges a legördülő menüből. 
- Majd pedig a markertábla paramétereit szükséges megadnunk. 
- Az bemeneti adatok megnyitására az "Open", míg a bementi adatok mentésére a "Save" gomb használható. Ha az inicializációval végeztünk akkor a továbblépéshez az "Accept" gomb lenyomása szükséges.
- Az inicializációs felületet a lentebbi ábra szemlélteti:

![alt text](https://github.com/ArminKaroly/Diploma/blob/master/Images/Initialization.png?raw=true)


####-- <ins>Kamerakalibráció</ins>
- A kamera kalibrálásához két lehetőségük van. Vagy manuális, vagy automata módon készütünk kalibrációs képeket.
- A manuális esetben a robotkar mozgatásához a megadott gombok állnak rendelkezésre, amelyek lenyomásával a robotkar Cartesian koordináták mentén vezérelhető. A képek készítéséhez a "Take picture" gomb szolgál. A "Show detected markers" lehetőség kiválasztása esetén láthatók a kameraképen a megtalált markerek. Ennek a kezelőfelületnek a bemutatására az alábbi kép szolgál: 
![alt text](https://github.com/ArminKaroly/Diploma/blob/master/Images/Manual.png?raw=true)


- Az automatikus képkészítő eljáráshoz szükséges több paraméter megadása. A gömb  amelyen a robotkar mozogni fog a "Radius" értékének módosításával adható meg. A gömbhálón méretét "width" és "length" paraméterek határozzák meg, míg a gömbháló pontjai közötti szögelfordulások értékét a "Horizontal distance" és "Vertical distance" paraméterekkel állíthatjuk.
- A felvett helyzetek orientációk szerinti bővítéséhez az "Orientation" részen belüli értékek köthetők. Ezek hasonlóan definiált értékek mint a gömbháló esetében. 
- A "Layers" részen belül a gömbfelületek száma módosítható, illetve a gömbfelületek közötti távolság adható meg. 
![alt text](https://github.com/ArminKaroly/Diploma/blob/master/Images/Automatic.png?raw=true)

- Az elkészített képeket a "Created Images" fülön belül találjuk ahol a készített képek törölhetők, és megtekinthetők. 
![alt text](https://github.com/ArminKaroly/Diploma/blob/master/Images/Created_images.png?raw=true)

- A kalibráció folyamatának elindításához a "Calculate camera parameters" gomb lenyomás szükséges.

- A kamerakalibrációs értékek beolvasása az "Open", míg azok mentése a "Save" gombal valósítható meg. Ha a kamerakalibrációs folyamattal végeztünk akkor a továbblépéshez az "Accept" gomb lenyomása szükséges. A teljes kezelőfelülelet a következő ábra szemlélteti:
![alt text](https://github.com/ArminKaroly/Diploma/blob/master/Images/CameraCalib.png?raw=true)

####-- <ins>Prekalculációs lépés</ins>
- Ebben a lépésben szükséges lemérnünk egy kezdeti becslést a robotkar bázisa és a tábla koordinátarendszere közötti transzformációhoz.
- Ennek a lemérése után a kezdeti becslés paraméterei beírandók a kezelőfelüt megfelelő helyeire. 
- Több kép készíthető a korábban bemutatott módszerekkel, ha a felhasználó ezt szükségesnek érzi.
- Az ismeretlen paraméterek számításához a "Calculate transformations" gomb megnyomás aszükséges.
- Hasonlóan mint korábban a szükséges értékek beolvashatók, a kiszámított értékek kimenthetők az "Open" és "Save" gombbal. Illetve a számítások elvégézse után az applikáció tovább léptethető az "Accept" gombbal. 
- Az applikáció felületét a következő ábra szemlélteti: 
![alt text](https://github.com/ArminKaroly/Diploma/blob/master/Images/PreCalculation.png?raw=true)

### -- <ins> Robotkalibráció </ins>
