void setup() {
  // put your setup code here, to run once:
  
}

void loop() {
  // put your main code here, to run repeatedly:

}


void fill_cb(void)
{
  if ( Serial.available() )
  {
    if ( cb.available() )
    {
      cb.write(Serial.read())
    }
  }
}


void rx_parser(void)
{
  if (state == "IDLE")
  {
    if (cb.read() == "<")
    {
      state = "CHECK_ST";
    }
    else
    {
      Serial.println("Error during parsing");
      state = "IDLE";
    }
  }
  if (state == "CHECK_ST")
  {
    if (cb.read() == 'S' && cb.read() == 'T' && cb.read() == '>')
    {
      state = "START_PARS";
    }
    else
    {
      Serial.println("Parse error");
      state = "IDLE";
    }
  }
  if (state == "START_PARS")
  {
    if ((c = cb.read()) == '<')
    {
      state = "CHECK_EN";
      buf.write('\0')
    }
    else
    {
      buf.write( c );
    }
  }
  if (state == "CHECK_EN")
  {
    if (cb.read() == 'E' && cb.read() == 'N' && cb.read() == '>')
    {
      state = "IDLE";
      cmd_complete = true;      //tells void loop() that a full cmd is available to execute
    }
    else
    {
      Serial.println("Parse error");
      state = "IDLE";
    }
  }
}

//  <ST>CTRL, m1, dir1, m2, dir2, t1, t2, ..., t8<EN>
//  cmd: 4 char + '\0' always
void cmd_exec (char buf[])
{
  char cmd[5];
  int i, j;
  for ( i = 0 && j = 0 ; j < 4; i++)
  {
    cmd[j] = buf[i];
  }
  cmd[j] = '\0';

  if ( cmd == "CTRL" )
  {
    int val[12] = {0};
    for ( int j = 0; j < sizeof(val)/sizeof(val[0]); j++ )
    {
      while ( buf[i++] != ',' && buf[i] != '\0' )
      {
        val[j] *= 10;
        val[j] = buf[i] - '0';
      }
    }

    analogWrite(PWM_PIN_A, val[0]);
    digitalWrite(DIR_PIN_A, val[1]);
    analogWrite(PWM_PIN_A, val[2]);
    digitalWrite(DIR_PIN_A, val[3]);
  }
}