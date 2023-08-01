star configuration  
stator should consist out of four or eight magnets;  
 result comes from https://things-in-motion.blogspot.com/2019/01/selecting-best-pole-and-slot.html  
 click on document, exclude q>0.5 and q<0.25 and slots is poles
 two options; 4 pole or 8 pole
 8 pole has higher torque but is able to achiever lower RPM  
carl uses custom angular magnets most likely ordered via aliexpress  
3 hall sensors guarantee that you lead at 90 degrees, don't think carl uses this  
duty cycle --> hard switching
six step commutation --> N N X S S X, each phase goes through this cycle

  
order new magnets  
don't fixate to prism --> carl also uses ordinary glue  

results so far;
  - speed of 10 Hertz
  - biggest problem is acrylic bearing 


optie 1: 
	buiten PTFE plaat, 3 mm dik 
	binnen PTFE plaat, 2 mm dik
	tussen prisma en binnen PTFE plaat leg je nog iets van 0.5 mm dik
	
optie 2:
    binnen en buiten PTFE plaat 2 mm dik
    plastic spacer of iets / geprint onder de buitenplaat voor offset van 0.5 mm
       
       
als je draait wil je de duty cycle tunen --> te hoog (versnelt en vertraagt ie)
                                         --> te laag (draait ie niet rond)
                                         
je kan ook het voltage aanpassen
                                         
kinetic friction is onhankelijk van de snelheid, zolang er geen trillingen onstaan
work done by friction schaalt wel met de weglengte.

je bouwt een rotatie energie op, hoe hoger het gewicht en hoe gladder het lager
hoe minder de impact van de kinetische wrijving, je magneten vangen alleen deze wrijving af


verder het gaat er om dat de laser op dezelfde tijd op het facet aankomt
 --> snelheid hoeft niet uniform te zijn, je kan desnoods een andere interpolatie kiezen

op dit moment heb je een kinetische wrijving van 0.4 (aclrylaat op acrylaat)
dit gaat naar 0.03

magneten zijn nu 2 mm dik
