<?/*Admin confirm Datenbank Lernmedien*//* verbinden mit db */		$db = mysql_connect("localhost","ruediheimlicher","RivChuv4");	/* nun ist der zugang zum db-server in der variable $db gespeichert */	mysql_set_charset('utf8',$db);	/* hier wird nun die eigentliche db ausgewählt */	mysql_select_db("ruediheimlicher_lernmedien", $db); ?><!DOCTYPE html PUBLIC "-//W3C//DTD XHTML 1.0 Strict//EN"        "http://www.w3.org/TR/xhtml1/DTD/xhtml1-strict.dtd"><html xmlns="http://www.w3.org/1999/xhtml" lang="de"><head>  <meta http-equiv="Content-Type" content="text/html; charset=UTF-8" /><title>Lernmedien Adminconfirm</title> <link href="lernmedien.css" rel="stylesheet" type="text/css" /></head><body class="liste">			<div><h1 class="lernmedien">Lernmedien</h1></div>	<h2 class="lernmedien">Admin confirm</h2>				<?#var_dump($_POST);$task =  $_POST['task'];$taskradio = $_POST['changeradio'];print 'taskradio: '.$taskradio.'<br>';$changeoption = $_POST['changeoption'];print 'changeoption: '.$changeoption.'<br>';$deleteangabe = $_POST['delete'];print 'deleteangabe: '.$deleteangabe.'<br>';$delangabe = $_POST['del'];print 'delangabe: '.$delangabe.'<br>';/*print 'task: '.$task.'<br>';$choose =  array($_POST['index']);#$choosestring = implode(', ',$choose);print 'choose: '.$choose[1][1].' count: '.count($choose[0]).'<br>';print_r($choose[0]);print '<br>';$index =  array($_POST['index']);print_r($index);print '<br>';print 'index: '.count($index).' zeile:  '.$index[0][0].'<br>';$test =  $_POST['test'];print 'test: '.$test.'<br>';*/if ($task == 'new'){	print'neuer Datensatz<br>';	$index = $_POST['index'];	$name = $_POST['neuername'];	$art = $_POST['neueart'];	$gruppe = $_POST['neuegruppe'];	$stufe = $_POST['neuestufe'];	$preis = $_POST['neuerpreis'];	$beschreibung = $_POST['neuebeschreibung'];		print '<p>index:*'. $index.'*</p>';	print '<p>name:*'. $name.'*</p>';	print '<p>art:*'. $art.'*</p>';	print '<p>gruppe:*'. $gruppe.'*</p>';	print '<p>stufe:*'. $stufe.'*</p>';	print '<p>preis:*'. $preis.'*</p>';	print '<p>beschreibung:*'. $beschreibung.'*</p>';		$eingabe=array($_POST['eingabe']);	#print_r($eingabe);		$name=mysql_real_escape_string($name);	$art=mysql_real_escape_string($art);	$gruppe=mysql_real_escape_string($gruppe);	$beschreibung=mysql_real_escape_string($beschreibung);		# Muster	#$result_insert = mysql_query("INSERT INTO lernmedien (id, name, beschreibung, art, gruppe, preis, stufe, US, MS, OS) VALUES (NULL, 'Abc', 'ysdfghjkl', 'CD', 'Holz', '1', 'US MS', '1', '1', NULL)");		$result_insert = mysql_query("INSERT INTO lernmedien (id, name, beschreibung, art, gruppe, preis, stufe, US, MS, OS) VALUES (NULL, '$name', '$beschreibung', '$art', '$gruppe', '$preis', '$stufe', NULL, NULL, NULL)");			$resultat=mysql_affected_rows($db);	print 'INSERT error: *'.mysql_error().'*<br>';	print 'resultat affected_rows: *'.$resultat.'*<br>';			print '<p>Rückgabe von INSERT: *'. $result_insert.'*</p>';	} # if newif ($task == 'change'){	print'Datensatz ändern<br>';	$index = $_POST['index'];	$name = $_POST['changename'];	$art = $_POST['changeart'];	$gruppe = $_POST['changegruppe'];	$stufe = $_POST['changestufe'];	$preis = $_POST['changepreis'];	$beschreibung = $_POST['changebeschreibung'];		print '<p>index:*'. $index.'*</p>';	print '<p>name:*'. $name.'*</p>';	print '<p>art:*'. $art.'*</p>';	print '<p>gruppe:*'. $gruppe.'*</p>';	print '<p>stufe:*'. $stufe.'*</p>';	print '<p>preis:*'. $preis.'*</p>';	print '<p>beschreibung:*'. $beschreibung.'*</p>';		$result_change = mysql_query("UPDATE lernmedien SET name = '$name', art = '$art', gruppe = '$gruppe', stufe = '$stufe',preis = '$preis',beschreibung = '$beschreibung'  WHERE id = '$index'");	$resultat=mysql_affected_rows($db);	print 'UPDATE error: *'.mysql_error().'*<br>';	print 'resultat affected_rows: *'.$resultat.'*<br>';	print '<p>Rückgabe von UPDATE: *'. $result_change.'*</p>';		} # if changeif ($task == 'delete'){	print'Datensatz loeschen<br>';	#$index = $_POST['index'];	#$result_delete = mysql_query("DELETE FROM   WHERE id = '$index'");	#$resultat=mysql_affected_rows($db);	#print 'DELETE error: *'.mysql_error().'*<br>';	#print 'resultat affected_rows: *'.$resultat.'*<br>';	#print '<p>Rückgabe von DELETE: *'. $result_change.'*</p>';	} # delete# Daten aendern# update students set first_name='Suba' where rec_id=678;#$db->set_charset("utf8"); # ******************************# Überprüfen# ******************************print '<h2 class="lernmedien">Admin Kontrolle</h2>';/* sql-abfrage schicken */$result_medien = mysql_query("SELECT * FROM lernmedien", $db);/* resultat in einer schleife auslesen */#print '<table width="759" border="1">';while ($medien = mysql_fetch_array($result_medien) ){	$x=$medien['name'];	print '<p>Das ist ein Eintrag. *'. $x.'*</p>';}# neuer Datensatz#print '<h2 class="lernmedien">neuer Datensatz</h2>';#$index=0;#mysql_data_seek($result_medien,$index);// Tableheader#$tableheaderstring  = '<th class="text" width="60">Name</th>';#$tableheaderstring  = '<th class="text breite60">Name</th>';#zu lerntest.pl#print '<form action="/cgi-bin/lerntest.pl" method="post" accept-charset="utf-8">';print '<br><form action="admin.php" ><p class="nameneingabe"><input type="submit" value="zurück" name="textfile"></p></form>';#zurueck#print '<form action="lernmedien.php" ><input type="submit" value="zurück" name="textfile"></form>';// probeweise#print $adressestring;/*perlscript aufrufen*/$index=0;mysql_data_seek($result_medien,$index);$zeile= mysql_fetch_assoc($result_medien);#print "<p>Ausgabe Zeile:</p>";#print '<p>*name: *'.$zeile['name'].'* art: *'.$zeile['art'].'* PREIS: *'.$zeile['preis'].'*</p>';?></div><?php# $para1 = "dies ist ein string";# $para2 = 42; # $pstring = sprintf("para1=%s&amp;para2=%s",#		urlencode($para1),#		urlencode($para2));?><!--<a href="benutzerauswahl.php?<?php print $pstring ?>">go</a> --><!--<p><a href="benutzerauswahl.php">senden</a></p>--><br><p class="liste">Verantwortlich f&uuml;r den Inhalt dieser Seiten ist:<br><a href="mailto:rosmarie.egli@bluewin.ch">Rosmarie Egli</a>	</p><br><br>	<p>	<?		?>	</p>			<p>		</p></body></html>