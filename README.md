Dies ist meine erweiterte Version vom Taupunktlüfter mit Logging-Funktion der Maker Media GmbH.
Das Original mit den wichtigsten Fragen rund um den Taupunktlüfter beantwortet eine **[FAQ](https://heise.de/-6526328)**. Die vollständigen Artikel zum Projekt gibt es in der **[Make-Ausgabe 1/22 ab Seite 22](https://www.heise.de/select/make/2022/1/2135511212557842576)** und in der **[Make-Ausgabe 2/22 ab Seite 82](https://www.heise.de/select/make/2022/2/2204711461516715363)** zu lesen.

Das Original Github Projekt liegt hier: https://github.com/MakeMagazinDE/Taupunktluefter

*Taupunktlüfter mit Datenlogger V3.0*

# Unterstützung von bis zu vier Tür-Sensoren. D.h. wenn der entsprechende Sensor
anzeigt, dass die Tür nicht geschlossen ist, springt auch der Lüfter nicht an.
Als Sensoren können einfache Schließer (z.B. Magnetschalter) zum Einsatz kommen.
Schalter geschlossen, bedeutet Tür geschlossen.
Auf dem PCB sind Jumper vorgesehen um die Schalter zu überbrücken.
Wird eine offene Tür erkannt, so geht einerseits die Hinweis-LED permanent an,
andererseits wird eine entsprechende Information auf dem Display ausgegeben.
# Display-Auto-Off
Das Backlight des Displays wird nach einer bestimmten Zeit deaktiviert. Um es wieder zu
aktivieren, ist ein dafür vorgesehener Taster zu drücken.
# Umbenennung der Sensoren
„Sensor 1“ und „Sensor 2“ wurden umbenannt nach „Sensor I“ für „Sensor innen“ und
„Sensor A“ für „Sensor außen“.
# Erstellung von Schaltplan und Layout in KiCad 6.0;
Platine passend für Gehäuse Maße: 200 x 120 x 57 mm Deckel klar
https://www.ebay.de/itm/312566082994?var=611287841843


