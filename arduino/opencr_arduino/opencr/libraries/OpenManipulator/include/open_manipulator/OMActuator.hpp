uint8_t* getPosition(uint8_t* (*get)(void))
{
  return get();
}

void setPosition()
{

}



getPosition(omDynamxiel.getPosition());