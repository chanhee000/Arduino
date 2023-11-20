/*size(1000,1000);
fill(255,0,0);//색 변형 코드
textSize(50);//글자 크기 변경 코드
background(255);
text("HELLO",450,500);
*/
/*
int x = -160;

void setup() {
  size(300, 100);
  fill(125, 0, 250);
  textSize(50);
}

void draw() {
  background(255); 
  text("HELLO", x, 50); // (x,50)인 곳에 hello를 출력합니다.
  x++;                  // x를 1씩 더합니다.
  if(x == width)
  {
    x = -160;
  }
}
*/
/* 
좌표 트랜스폼 이용한 도형 그리기
void setup()
{
  size(1000,1000);
  background(255);
  rect(0,0,150,150);
  translate(400,400);
  rect(0,0,150,150);
}
*/
void setup()
{
  size(500,500);
  background(255);
  //rect(0,0,150,150);
  translate(200,200);
  rect(0,0,150,150);
  translate(-200,-200);
  rect(50,50,150,150);
}
/*
/집만들기
//house2(i*a,b);            a = 집 사이의 간격 b = 집의 y 축
void setup()
{
  size(1000,1000);
  background(255);
  for(int i = 0; i<4;i++)
  {
    house1(i*150,100);
    house2(i*150,300);
  }
}

void house1(int x, int y)
{
  fill(0);
  triangle(x,y,x+100,y,x+50,y-30);
  fill(100);
  rect(x,y,100,100);
}
void house2(int x,int y)
{
  pushMatrix();
  println(x,y);
  translate(x,y);
  fill(200,50,0);
  triangle(0,0,100,0,50,-30);
  fill(50,100,200);
  rect(0,0,100,100);
  popMatrix();
}
*/
