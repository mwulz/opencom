/* Ansteuerung Conrad-Relaisplatine Best. Nr. 96 77 20
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 */
 
/* Zum Debuggen DEBUG definieren, d.h. in der folgenden
 * Zeile die Kommentare entfernen
 */
/* #define DEBUG */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>   /* File control definitions */
#include <termios.h> /* POSIX terminal control definitions */

/*
 * Befehlssatz der Karte:
 * <x> = 'dont't care', in der Regel 0
 * XOR = Pruefinfo (XOR aller Werte, wird vom Programm berechnet)
 *
 * Kommando                        Kommandorahmen      Antwort
 * -------------------------------------------------------------------------
 * 0  NO OPERATION - keine Aktion  0 <Adr.> <x> XOR    255 <Adr.>  <x> XOR
 * 1  SETUP - Initialisierung      1 <Adr.> <x> XOR    254 <Adr.> <info> XOR
 * 2  GET PORT - Schaltzustand     2 <Adr.> <x> XOR    254 <Adr.> <info> XOR
 * 3  SET PORT - Relais schalten   3 <Adr.> <data> XOR 253 <Adr.> <data> XOR
 * 4  GET OPTION - Option lesen    4 <Adr.> <x> XOR    254 <Adr.> <info> XOR
 * 5  SET OPTION - Option setzen   5 <Adr.> <data> XOR 254 <Adr.> <data> XOR
 * -------------------------------------------------------------------------
 *
 * Das Init-Kommando initialisiert alle Karten einer Kette. Der Computer
 * erhaelt also bei N Karten N+1 Antworten! Dasselbe gilt fuer
 * das GET PORT-Kommando.
 * Bei den Optionen koennen nur "enable broadcast" und "block broadcast"
 * gesetzt werden:
 *   Wert  Broadcast  Broadcast
 *        ausfuehren  blockieren
 *  ---------------------------------------------
 *    0     nein         nein
 *    1      ja          nein   (Voreinstellung)
 *    2     nein          ja
 *    3      ja           ja    (nicht sinnvoll) 
 *  ---------------------------------------------
 * Bei blockierten Brodcast wird bei INIT und GET PORT nur ein NOP an
 * die nachfolgenden Platinen gesendet.
 * 
 * Ausgangsport ist normalerweise ttS0 bis ttyS3.
 * Der Port muss fuer den user oder die Gruppe, unter der
 * das Programm laeuft, Schreibberechtigung haben.
 * Die User des Programms koennen in die Gruppe von ttySx 
 * aufgenommen werden (meist 'tty' oder 'dialout').
 * Gegebenfalls kann man auch das Programm der Gruppe von ttySx 
 * zuordnen und das SetGroupId-Bit setzen.
 */

#define PORT 0

int open_port(int port) 
  {
  /*
   * Oeffnet seriellen Port 
   * Gibt das Filehandle zurueck oder -1 bei Fehler
   * der Parameter port muss 0, 1, 2 oder 3 sein 
   *
   * RS232-Parameter
   * - 19200 baud
   * - 8 bits/byte
   * - no parity
   * - no handshake
   * - 1 stop bit
   */
   int fd;
   struct termios options;
   switch (port)
     {
     case 0: fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY); break;
     case 1: fd = open("/dev/ttyS1", O_RDWR | O_NOCTTY | O_NDELAY); break;
     case 2: fd = open("/dev/ttyS2", O_RDWR | O_NOCTTY | O_NDELAY); break;
     case 3: fd = open("/dev/ttyS3", O_RDWR | O_NOCTTY | O_NDELAY); break;
     default: fd = -1;
     }
   if (fd >= 0)
     {
     /* get the current options */
     fcntl(fd, F_SETFL, 0);
     if (tcgetattr(fd, &options) != 0) return(-1);
   
     cfsetispeed(&options, B19200);            /* setze 19200 bps */
     cfsetospeed(&options, B19200);
     /* setze Optionen */ 
     options.c_cflag &= ~PARENB;               /* kein Paritybit */
     options.c_cflag &= ~CSTOPB;               /* 1 Stopbit */
     options.c_cflag &= ~CSIZE;                /* 8 Datenbits */
     options.c_cflag |= CS8; 
     options.c_cflag |= (CLOCAL | CREAD);      /* CD-Signal ignorieren */
     /* Kein Echo, keine Steuerzeichen, keine Interrupts */
     options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
     options.c_oflag &= ~OPOST;                /* setze "raw" Input */
     options.c_cc[VMIN]  = 0;                  /* warten auf min. 0 Zeichen */
     options.c_cc[VTIME] = 10;                 /* Timeout 10 Sekunden */
     tcflush(fd,TCIOFLUSH);
     if (tcsetattr(fd, TCSAFLUSH, &options) != 0) return(-1);

/*
     fcntl(fd, F_SETFL, FNDELAY);
     fcntl(fd, F_SETFL, 0);
*/
     }
  return(fd);
  }


int sndcmd(int fd, unsigned char command, unsigned char addr, 
             unsigned char data) 
  {
  /*
   * Sendet Kommando an die Relaiskarte
   * die Berechnung der Pruefsumme erfolgt automatisch
   * Rueckgabe: 0 bei Erfolg, -1 bei Fehler
   */
  unsigned char wbuf[4];

  wbuf[0] = command;
  wbuf[1] = addr;
  wbuf[2] = data;
  /* Pruefsumme */
  wbuf[3] = wbuf[0]^wbuf[1]^wbuf[2];
  #ifdef DEBUG
  printf(" -> send: %d %d %d %d\n", wbuf[0], wbuf[1], wbuf[2], wbuf[3]);
  #endif
  usleep(50000L); /* 50 ms Pause */
                  /* Verhindert "Flattern" bei zu schnellem Wechsel */
  if (write(fd, wbuf, 4)) return(0);
  else return(-1);
  }


int rcvstat(int fd, unsigned char *answer, unsigned char *addr, 
             unsigned char *data) 
  {
  /*
   * Empfaengt Status von der Relaiskarte
   * die Berechnung der Pruefsumme erfolgt automatisch
   * Rueckgabe: 0 bei Erfolg, -1 bei Fehler
   */
  unsigned char rbuf[4];
  int xor;
  read(fd, rbuf, 4);
  xor = rbuf[0]^rbuf[1]^rbuf[2];
  #ifdef DEBUG
  printf(" -> recv: %d %d %d %d\n", rbuf[0], rbuf[1], rbuf[2], rbuf[3]);
  #endif
  *answer = rbuf[0];
  *addr = rbuf[1];
  *data = rbuf[2];
  if (xor == rbuf[3]) return(0);
  else return(-1);
  }

void help(void)
  {
  /* Hilfetext ausgeben */
  fprintf(stderr,"Ansteuerprogramm fuer die serielle Relaiskarte von Conrad\n");
  fprintf(stderr,"No Parameters!\n");
  fprintf(stderr,"\n");
  fprintf(stderr,"Usage: relais <Parameter>\n");
  fprintf(stderr,"\n");
  fprintf(stderr,"Parameter:\n");
  fprintf(stderr,"   -stat:  Status der Relais als Dezimalzahl\n");
  fprintf(stderr,"           Bit=1: Relais an, Bit=0: Relais aus\n");
  fprintf(stderr,"           keine weiteren Parameter moeglich\n");
  fprintf(stderr,"   -off:   alle Relais aus\n");
  fprintf(stderr,"   -on:    alle Relais an\n");
  fprintf(stderr,"   -sx:    Relais x einschalten (1 <= x <= 8)\n");
  fprintf(stderr,"   -rx:    Relais x ausschalten (1 <= x <= 8)\n");
  fprintf(stderr,"\n");
  fprintf(stderr,"Beispiel:\n");
  fprintf(stderr,"   relais -off -s1 -s3: Relais 1 und 3 einschalten\n");
  fprintf(stderr,"   relais -s4 -r3:      Relais 4 ein- und 3 ausschalten\n");
  fprintf(stderr,"   relais -on -r6:      alle Relais ausser 6 einschalten\n");
  fprintf(stderr,"\n");
  fprintf(stderr,"Rueckgabewert:\n");
  fprintf(stderr,"OK: <Kontaktstellung dezimal>\n");
  fprintf(stderr,"z. B. 'OK: 5' --> Relais 1 und 3 on\n");
  fprintf(stderr,"Bei Fehler wird FAIL: und der komplette Status\n");
  fprintf(stderr,"zurueckgegeben (Antwortcode Adresse Daten/Info).\n");
  fprintf(stderr,"\n");
  }


int main(int argc, char *argv[])
  {
  int fd;                 /* File descriptor for the port */
  unsigned char ans, adr, stat, val, rval, n;

  fd =  open_port(PORT);
  if (fd == -1)
    {
    fprintf(stderr,"Cannot open port %d\n",PORT);
    exit(1);
    }

  if ((argc <= 1) || (strcmp(argv[1],"--help") == 0))
    {
    help();
    exit(1);
    }

  /* Karte Initialisieren */
  sndcmd(fd, 1, 1, 0);
  val = rcvstat(fd,&ans,&adr,&stat);
  if ((val != 0) || (ans != 254))
    {
    fprintf(stderr,"FAIL: card init failed\n");
    exit(1);
    }
 /* zweite Antwort vernichten */
  val = rcvstat(fd,&ans,&adr,&rval);
  
  /* Aktuellen Stand abfragen */
  sndcmd(fd, 2, 1, 0);
  val = rcvstat(fd,&ans,&adr,&rval);
  while ((val == 0) && (ans != 253))
    {
    val = rcvstat(fd,&ans,&adr,&rval);
    }

  /* Parameter auswerten */
  for (n = 1; n < argc; n++)
    {
    if (strcmp(argv[n],"-stat") == 0)
      {
      /* Status liefern */
      if (ans == 253) printf("OK: %d\n", rval);
      else            printf("FAIL: %d %d %d\n", ans, adr, rval);
      close(fd);
      exit(0);
      }
    else if (strcmp(argv[n],"-off") == 0)
      {
      /* Alles ausschalten */
      rval = 0;
      }
    else if (strcmp(argv[n],"-on") == 0)
      {
      /* Alles einschalten */
      rval = 255;
      }
    else if ((argv[n][0] == '-') && (argv[n][1] == 's'))
      {
      val = atoi(&argv[n][2]);   /* Zahl hinter dem "-s" umwandeln */
      if ((1 <= val) && (val <= 8))
        {
        val = 1 << (val-1);
        rval = rval | val;
        }
      }
    else if ((argv[n][0] == '-') && (argv[n][1] == 'r'))
      {
      val = atoi(&argv[n][2]);   /* Zahl hinter dem "-r" umwandeln */
      if ((1 <= val) && (val <= 8))
        {
        val = ~(1 << (val-1));
        rval = rval & val;
        }
      }
    else
      {
      fprintf(stderr,"Wrong Parameter: %s\n",argv[n]);
      }
    }
  sndcmd(fd, 3, 1, rval);
  rcvstat(fd,&ans,&adr,&stat);
  if (ans == 252) printf("OK: %d\n", stat);
  else            printf("FAIL: %d %d %d\n", ans, adr, stat);

  close(fd);
  return 0;
  }


