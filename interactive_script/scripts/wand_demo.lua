d = 1.5
g = gesture('roll_right')
moveTo(g.x + d * math.cos(g.psi),
       g.y + d * math.sin(g.psi), g.z, g.psi)
wait()
