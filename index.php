<?/*Datenbank Lernmedien*//* verbinden mit db */		$db = mysql_connect("localhost","ruediheimlicher","RivChuv4");	/* nun ist der zugang zum db-server in der variable $db gespeichert */	mysql_set_charset('utf8',$db);	/* hier wird nun die eigentliche db ausgewählt */	mysql_select_db("ruediheimlicher_lernmedien", $db); ?><!DOCTYPE html PUBLIC "-//W3C//DTD XHTML 1.0 Strict//EN"        "http://www.w3.org/TR/xhtml1/DTD/xhtml1-strict.dtd"><html xmlns="http://www.w3.org/1999/xhtml" lang="de"><head>  <meta http-equiv="Content-Type" content="text/html; charset=UTF-8" /><title>Lernmedien</title> <link href="lernmedien.css" rel="stylesheet" type="text/css" /></head><body class="lernmedien">	<div class="titelfoto">	<img class="imgtitelfoto" src="Bilder/Fotos/titelblattfoto.jpg"  alt="Icon" ></div><h1 class="lernmedien">Lernmedien</h1>	<div class="menuabschnitt">		<table class="menutabelle">	<tr>	<td class="lernmedientabellentext "><a href="liste.php">Liste</a></td>		</tr>	<tr>	<td class="lernmedientabellentext "><a href="benutzerauswahl.php">Bestellung</a></td>		</tr>		</table>	</div>					<div class="abschnitt1">		<div class="gruppentabelle">				<?	# Werte aus Datenbank für die Artikel auslesen	$result_medien = mysql_query("SELECT * FROM lernmedien", $db);			$result_gruppen = mysql_query("SELECT gruppe, id FROM lernmedien", $db);	print mysql_error();	$gruppeindex=0;	$gruppenliste =array();		while ($gruppen = mysql_fetch_row($result_gruppen) )	{		$zeile=$gruppen[0];		#print '<p>Eintrag: '.$gruppeindex.' Er ist: *'. $zeile.'*</p>';		$trenner=" ";		$elemente=explode($trenner,$zeile);		foreach($elemente AS $element)		{			#print '<p>Inhalt von Zeile: '.$gruppeindex.': *'. $element.'*<br></p>';			if (!(in_array($element,$gruppenliste,0)))			{				$gruppenliste[]=$element;						}				}		$gruppeindex++;	}		$zeile=0;	$index=0;	print '<table class="lernmedientabelle">';	#print '<tr >';		foreach($gruppenliste AS $gruppenelement)	{		#Gruppenliste aus Datenbank		#print '<p>gruppenliste index: '.$zeile.' element: '.$gruppenelement.'<br></p>';		if ($zeile%2 ==0)		{			# neue Zeile anfangen			print '	<tr>';		}		else		{			# Trennfeld einfügen			print '<td class="lernmedientabellentext oben breite30"></td>';		}				$ordnername=$gruppenliste[$zeile];		$bildnummer="001";		$bildlink = "Bilder/Gruppen/".$ordnername."/".$bildnummer.".jpg";		#print '<p>bildlink: '.$bildlink.' von Element:'. $ordnername.'*<br></p>';						print '<td class="lernmedientabellentext oben breite200">' . '<strong>'.$ordnername . '</strong><br>';		#print $medien['beschreibung'] . '<br>';		#print 'Art: '.$medien['art'] . '<br>';		#print 'Stufe: '.$medien['stufe'] . '<br>';		#print '<strong>Preis: ' . $medien['preis'] . '</strong><br>';		#print 'Bestellen: '.'<input type = "text" name= "eingabe[]"  size="1" maxlength="2"></td>';			#print '<td class="lernmedientabellentext oben breite200">' . '<strong>'.$medien['name'] . '</strong><br>';		print '<td class="lernmedientabellenbild breite200"><img class="mediumfoto" src='. $bildlink .' alt="Icon" ></td>';			if ($zeile%2 ==1)		{			print '	</tr>';		}		$zeile++;	}		print '</table>';	/* sql-abfrage schicken */		/* resultat in einer schleife auslesen */			#print '<table width="759" border="1">';	#print '<table margin-bottom="20px">';	#print '<tr >';		#while ($medien = mysql_fetch_array($result_medien) )	{		#$x=$medien['name'];		#print '<p>Das ist ein Eintrag. Er ist: '. $x.' Punkt</p>';	}	#$index=0;		#mysql_data_seek($result_medien,$index);					# zu benutzerauswahl	print '<form action="benutzerauswahl.php" method="post">';	#print '<form action="/cgi-bin/lerntest.pl" method="post" accept-charset="UTF8">';			#print '<input type="hidden" name=index[] value ='.$medien['id'].'>';	print '<br><p class="liste" ><input type="submit" name="Bestellung" value="zur Bestellung"></p></form>';	#zurueck	#print '<form action="lernmedien.php" ><input type="submit" value="zurück" name="textfile"></form>';	// probeweise	#print $adressestring;		/*perlscript aufrufen*/	$index=0;	mysql_data_seek($result_medien,$index);	$zeile= mysql_fetch_assoc($result_medien);	#print "<p>Ausgabe Zeile:</p>";	#print '<p>*name: *'.$zeile['name'].'* art: *'.$zeile['art'].'* PREIS: *'.$zeile['preis'].'*</p>';	print '<br><form action="http://web.me.com/ruedi.heimlicher/Website/Lernmedien.html" ><p class="liste"><input type="submit" value="zurück" name="textfile"></p></form>';		?>	</div></div><?php# $para1 = "dies ist ein string";# $para2 = 42; # $pstring = sprintf("para1=%s&amp;para2=%s",#		urlencode($para1),#		urlencode($para2));?><!--<a href="benutzerauswahl.php?<?php print $pstring ?>">go</a> --><!--<p><a href="benutzerauswahl.php">senden</a></p>--><div class="abschnitt2"><br><form action="admin.php" ><p class="liste"><input type="submit" value="admin" name="admin"></p></form><br><form action="http://web.me.com/ruedi.heimlicher/Website/Lernmedien.html" ><p class="liste"><input type="submit" value="zurück" name="textfile"></p></form>	<br><p class="liste">Verantwortlich f&uuml;r den Inhalt dieser Seiten ist:<br>	<a href="mailto:rosmarie.egli@bluewin.ch">Rosmarie Egli</a>	</p></div><br><br>	<p>	<?		?>	<?		?>	</p></body></html>