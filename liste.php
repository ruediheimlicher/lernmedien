<?/*Datenbank Lernmedien*//* verbinden mit db */		$db = mysql_connect("localhost","ruediheimlicher","RivChuv4");	/* nun ist der zugang zum db-server in der variable $db gespeichert */	mysql_set_charset('utf8',$db);	/* hier wird nun die eigentliche db ausgewählt */	mysql_select_db("ruediheimlicher_lernmedien", $db); ?><!DOCTYPE html PUBLIC "-//W3C//DTD XHTML 1.0 Strict//EN"        "http://www.w3.org/TR/xhtml1/DTD/xhtml1-strict.dtd"><html xmlns="http://www.w3.org/1999/xhtml" lang="de"><head>  <meta http-equiv="Content-Type" content="text/html; charset=UTF-8" /><title>Lernmedien</title> <link href="lernmedien.css" rel="stylesheet" type="text/css" /></head><body class="liste">			<div><h1 class="lernmedien">Lernmedien</h1></div>	<h2 class="lernmedien">Liste</h2>				<?function leerschlagweg($string)	{		$newstring=preg_replace('/ /', '_',$string);		return $newstring;	}			function umlautweg($string)	{		# $ccc = hash{"ä"=>"ae", "ö"=>"oe", "ü"=>"ue", "Ä"=>"Ae", "Ö"=>"Oe", "Ü"=>"Ue"};		#s/([äüöÄÜÖ]/$ccc{$1}/g;		#$newstring =~ s/([äüöÄÜÖ]/$ccc{$1}/g;		$newstring=preg_replace('/ä/', 'ae',$string);		$newstring=preg_replace('/ö/', 'oe',$newstring);		$newstring=preg_replace('/ü/', 'ue',$newstring);		return $newstring;	}$adressestring = '<h1 class="lernmedien">Adresse</h1>';$adressestring .= '<form action="/cgi-bin/lernmedienbestellung.pl" method="post">';$adressestring .= '<p class="nameneingabe">Name:<br><input size="40" maxlength="40" name="AnwenderName"></p>';$adressestring .= '<p>Text:<br><textarea rows="5" cols="50" name="Kommentartext"></textarea></p>';$adressestring .= '<p><input type="submit" value="Absenden"></p>';$adressestring .= '</form>';#$db->set_charset("utf8"); /* sql-abfrage schicken */$result_medien = mysql_query("SELECT * FROM lernmedien", $db);/* resultat in einer schleife auslesen */#while ($medien = mysql_fetch_array($result_medien) ){	#$x=$medien['name'];	#print '<p>Das ist ein Eintrag. Er ist: '. $x.' Punkt</p>';}#$index=0;#mysql_data_seek($result_medien,$index);print '<table margin-bottom="20px">';print '<tr >';// Tableheader#$tableheaderstring  = '<th class="text" width="60">Name</th>';#$tableheaderstring  = '<th class="text breite60">Name</th>';$tableheaderstring  = '<th class="text breite150">Name</th>';$tableheaderstring .= '<th class="text ">Beschreibung</th>';$tableheaderstring .= '<th class="icon">Bilder</th>';$tableheaderstring .= '<th width="60" align="left" ><p class="tableheader">Art</p></th>';$tableheaderstring .= '<th width="60" align="center" ><p class="tableheader">Stufe</p></th>';$tableheaderstring .= '<th width="80" align="center" ><p class="tableheader">Gruppe</p></th>';$tableheaderstring .= '<th width="50" align="center" ><p class="tableheader">Preis</p></th>';#$tableheaderstring .= '<th width="50" align="center" ><p class="tableheader">Icon</p></th>';$tableheaderstring .= '<th width="10" align="center" ><p class="tableheader">Anzahl</p></th>';$tableheaderstring .= '	</tr>';print $tableheaderstring;#zu lerntest.pl#print '<form action="/cgi-bin/lerntest.pl" method="post" accept-charset="utf-8">';# zu benutzerauswahlprint '<form action="benutzerauswahl.php" method="post">';#print '<form action="/cgi-bin/lerntest.pl" method="post" accept-charset="UTF8">';$zeile=0;while ($medien = mysql_fetch_array($result_medien) ){	print '<tr >';	//print '<td align="center">'.'<input type = "hidden" name= ' . $medien['id'] . '>';	$ordnernummer=sprintf("%03d",$medien['id']);	$bildnummer=$ordnernummer."0010";	$bildlink = "Bilder/".$ordnernummer."/".$ordnernummer."0010".".jpg";	print '<td class="listetabellentext">' . $medien['name'] . '</td>';	#print '<td class="lernmedientabellenicon">' .$medien['beschreibung']. '<img src='. $bildlink .' alt="Icon" />' .'</td>';	print '<td class="listetabellentext">' . $medien['beschreibung'] . '</td>';	#print '<td class="lernmedientabellenicon"><img src='. $bildlink .' alt="Icon" >'.'<a href="Bilder/index.html">></a>'. '</td>';	$auswahlgruppe=$medien['gruppe'];	$webname=$medien['name'];	#print 'webname vor: '.$webname;	$webname=leerschlagweg($webname);	#print 'webname nach leerschlagweg: '.$webname;	$webname=umlautweg($webname);	#print 'webname nach umlautweg: '.$webname.'<br>';	$ordnerpfad='Bilder/Gruppen/'.$auswahlgruppe.'/'.$webname;	print '<td class="listetabellenicon"><a href=detailanzeige.php'.'?ordnerpfad='.$ordnerpfad.' target="_blank">Details</a></td>';			#print '<td class="listetabellenicon"><img class="icon" src='. $bildlink .' alt="Icon" ></td>';	#print '<td class="lernmedientabellenicon">' . $bildlink . '</td>';		print '<td class="listetabellentext">' . $medien['art'] . '</td>';	print '<td class="listetabellentext">' . $medien['stufe'] . '</td>';	print '<td class="listetabellentext">' . $medien['gruppe'] . '</td>';	print '<td class="listetabellentext">' . $medien['preis'] . '</td>';	#<img src="tanzmaus.png" alt="Tanzmaus">		#print '<td align="center">'.'<a href="Bilder/index.html">>></a>' . '</td>';	print '<td align="center">'.'<input type = "text" name= "eingabe[]"  size="1" maxlength="2">';	print '	</tr>';		print '<input type="hidden" name=index[] value ='.$medien['id'].'>';	print '<input type="hidden" name=name[] value ="'.$medien['name'].'">';	print '<input type="hidden" name=preis[] value ='.$medien['preis'].'>';$zeile++;}print '</table>';#print '<input type="hidden" name=index[] value ='.$medien['id'].'>';print '<br><p class="liste" ><input type="submit" name="Bestellung" value="zur Bestellung"></p></form>';#zurueck#print '<form action="lernmedien.php" ><input type="submit" value="zurück" name="textfile"></form>';// probeweise#print $adressestring;/*perlscript aufrufen*/$index=0;mysql_data_seek($result_medien,$index);$zeile= mysql_fetch_assoc($result_medien);#print "<p>Ausgabe Zeile:</p>";#print '<p>*name: *'.$zeile['name'].'* art: *'.$zeile['art'].'* PREIS: *'.$zeile['preis'].'*</p>';print '<br><form action="index.php" ><p class="liste"><input type="submit" value="zurück" name="textfile"></p></form>';?></div><?php# $para1 = "dies ist ein string";# $para2 = 42; # $pstring = sprintf("para1=%s&amp;para2=%s",#		urlencode($para1),#		urlencode($para2));?><!--<a href="benutzerauswahl.php?<?php print $pstring ?>">go</a> --><!--<p><a href="benutzerauswahl.php">senden</a></p>--><br><p class="liste">Verantwortlich f&uuml;r den Inhalt dieser Seiten ist:<br><a href="mailto:rosmarie.egli@bluewin.ch">Rosmarie Egli</a>	</p><br><br>	<p>	<?		?>	</p>			<p>		</p></body></html>