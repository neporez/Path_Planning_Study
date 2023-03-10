# APF(Artificial Potential Field)



기본적인 개념은 도착점에서 차량을 끌어당기는 인력과 일정거리 안의 장애물이 차량을 밀어내는 척력으로 인해 경로가 구성된다.



![Screenshot from 2023-02-21 15-37-58](https://user-images.githubusercontent.com/88701811/220266705-a57ad78a-343c-4c52-837d-3fbd111e5be2.png)

![Screenshot from 2023-02-21 15-38-27](https://user-images.githubusercontent.com/88701811/220266956-8b519307-505f-4b70-9f76-c8748613af8c.png)


다음과 같은 Fatt와 Frep의 합으로 이루어지게 된다.


코드상으로 보면 이렇다.

<pre>
<code>
att = (self.goal - self.current_pos) * self.k_att

rep += Vector2d(t_vec.direction[0], t_vec.direction[1]) * self.k_rep * (1.0 / t_vec.length - 1.0 / self.rr) / (t_vec.length ** 2)
                        
f_vec = self.attractive() + self.repulsion()

</code>
</pre>

APF의 단점으로는

- 지역최소점(local minima)
- GNRON
- Dynamic Environment

으로 뽑을 수 있다.

# IAPF(Improved Artificial Potential Field)


APF의 단점을 보완하기 위해서 개발된 방법



위의 코드는 APF의 3가지 단점중 GNRON(장애물이 목표 근처에 있을 시 도달할 수 없는 문제)를 해결한 코드이다.



![Screenshot from 2023-02-21 15-56-22](https://user-images.githubusercontent.com/88701811/220270201-439b79ca-39a9-4df7-8963-a123bd9ac806.png)



기존의 APF와 비교했을 때 Fatt는 변하지 않고 Frep의 식에서 차량에서 도착점까지의 거리에 대한 식이 추가되었다.



코드 상으로 보면 이렇다.


<pre>
<code>
att = (self.goal - self.current_pos) * self.k_att

rep_1 = Vector2d(obs_to_rob.direction[0], obs_to_rob.direction[1]) * self.k_rep * (
                        1.0 / obs_to_rob.length - 1.0 / self.rr) / (obs_to_rob.length ** 2) * (rob_to_goal.length ** 2)
                rep_2 = Vector2d(rob_to_goal.direction[0], rob_to_goal.direction[1]) * self.k_rep * ((1.0 / obs_to_rob.length - 1.0 / self.rr) ** 2) * rob_to_goal.length
                rep +=(rep_1+rep_2)
                        
f_vec = self.attractive() + self.repulsion()

</code>
</pre>

