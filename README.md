# ATmega128_Elevator_Simul
ATmega128을 활용한 엘레베이터 시뮬레이션 임베디드 프로젝트입니다.

## 기능
1. 랜덤으로 현재 엘리베이터의 층수, 사용자의 위치 생성
2. FND 상위 두자리는 엘리베이터1, 하위 두자리는 엘리베이터2의 위치
(두 FND는 “.”으로 분리된다.)
3. LED로 현재 사용자가 있는 층을 나타냄
4. 스위치 두 개는 각각 엘리베이터1과 엘리베이터2를 사용자의 위치로 부른다.
5. 엘리베이터가 사용자의 층에 도달하였을 때, 버저는 도착음을 울린다.
6. 5번이 수행되면 사용자가 엘리베이터에 탔다고 가정하고, 
해당 엘리베이터는 랜덤 목적지를 설정하여 목적지 층으로 움직이게 된다.
(해당 FND값 변경)
7. 일정 온도에 도달하면 버저에서 비상음을 울린다. (화재 상황을 가정)

## Task
FndDisplayTask: 엘리베이터의 위치를 받아서 FND에 출력한다.

StartTask: 엘리베이터의 위치를 결정하고 엘리베이터 위치를 변경하는 역할이다.

TemperatureTask: 움직여야하는 상황일 때, ReadTemperature 함수를 통해 온도를 측정하고 사용자의 위치를 보여주기 위해 LedTask를 호출한다. 

moveAfter: 엘리베이터가 현재 층에 도착 하여 사용자를 태운 후, 사용자가 이동할 위치를 랜덤하게 설정한 상황을 가정하고, 랜덤 값을 만들어 엘리베이터의 이동 지점을 설정 해준다.
