
   void SetRelays(void)
 {
    //use either "relay" for sections, or "uTurn" for uTurn signals and relays
    if (bitRead(relay,0)) bitSet(PORTD, 5); //Digital Pin 5
    else bitClear(PORTD, 5); 
    if (bitRead(relay,1)) bitSet(PORTD, 6); //Digital Pin 6
    else bitClear(PORTD, 6); 
    if (bitRead(relay,2)) bitSet(PORTD, 7); //Digital Pin 7
    else bitClear(PORTD, 7); 
    if (bitRead(relay,3)) bitSet(PORTB, 0); //Digital Pin 8
    else bitClear(PORTB, 0); 
    if (bitRead(relay,4)) bitSet(PORTB, 1); //Digital Pin 9
    else bitClear(PORTB, 1); 
    if (bitRead(relay,5)) bitSet(PORTB, 2); //Digital Pin 10
    else bitClear(PORTB, 2); 
    if (bitRead(relay,6)) bitSet(PORTB, 3); //Digital Pin 11
    else bitClear(PORTB, 3); 
    if (bitRead(relay,7)) bitSet(PORTB, 4); //Digital Pin 12
    else bitClear(PORTB, 4); 

  }
  
