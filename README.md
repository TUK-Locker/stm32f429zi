![ㅍㅅㅌ](https://github.com/TUK-Locker/stm32f429zi/blob/main/iamges/%ED%8F%AC%EC%8A%A4%ED%84%B0.jpg)

## 소개
한국공학대학교 메카트로닉스공학과 졸작입니다.  
주제는 출납이 편리한 스마트 라커이며, 이 repository는 mcu개발 부분 F/W만 보여줍니다.  
팀원은 총 4명이며 하드웨어는 2명의 팀원들이 만들었으며 저는 mcu를 개발 하였고, 나머지 한명은 APP을 개발 했습니다.  

  
- 개발 기간 : 2022.03 ~ 2022.09  
- 사용 ide : stm32fcubeide  
- 사용 mcu : NUCLEO144(stm32f429zi)  
- 개발자 : 정인환




## 시연연상  
https://user-images.githubusercontent.com/90883626/224680719-9da6041c-789e-425c-998c-5804a09d7698.mp4


## 구조설명
총 두개의 뉴클레오 보드(제어기)가 사용되었으며,  
TUK_Car_2017132035에는 물건운반차량에 있는 제어기에 필요한 코드가,  
TUK_Elevator_2017132035에는 엘리베이터에 있는 제어기에 필요한 코드가 있습니다.  
추가로 엘리베이터에는 Wi-Fi 모듈인 esp8266을 연결하여 위에 필요한 코드도 있습니다.
