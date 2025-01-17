import pygame, sys
import pygame.event as EVENTS
import pygame.locals as LOCALS

class Information:


    def __init__(self, max_charge):
        pygame.init()
        self.font = pygame.font.SysFont("Comic Sans MS", 30)
        self.window_w = 900
        self.window_h = 900
        self.window = pygame.display.set_mode((self.window_w, self.window_h))
        pygame.display.set_caption("Information MoRoomba")
        
        self.state = ""
        self.map = []
        self.current_position = []
        self.orientacion = "Up"
        self.charge = 200
        self.max_charge = str(max_charge)


    def map_state(self, state):
        if state == "mapeando":
            return "Mapping"
        if state == "recargando":
            return "Charging"
        if state == "limpiando":
            return "Cleaning"
        return state


    def map_orientacion(self, orientacion):
        if orientacion == "izquierda":
            return "Left"
        if orientacion == "derecha":
            return "Right"
        if orientacion == "arriba":
            return "Up"
        if orientacion == "abajo":
            return "Down"
        return orientacion

    #Close the screen
    def quit(self):
        pygame.quit()
        sys.exit()
    
    def paint_map(self):
        cell_size = 15
        for i in range(len(self.map)):
            for j in range(len(self.map[0])):
                coord_x = cell_size * j - ((len(self.map[0]) * cell_size - self.window_w)/2)
                coord_y = cell_size * i - ((len(self.map) * cell_size - self.window_h)/2)
                if self.map[i][j] == 1:
                    pygame.draw.rect(self.window, (0,0,0), (coord_x, coord_y, cell_size, cell_size), 0)
                elif self.map[i][j] == 0:
                    pygame.draw.rect(self.window, (255,0,0), (coord_x, coord_y, cell_size, cell_size), 0)
                elif self.map[i][j] == 2:
                    pygame.draw.rect(self.window, (0,255,0), (coord_x, coord_y, cell_size, cell_size), 0)
                elif self.map[i][j] == 3:
                    pygame.draw.rect(self.window, (0,0,255), (coord_x, coord_y, cell_size, cell_size), 0)
        
        coord_x = cell_size * self.current_position[1] - ((len(self.map[0]) * cell_size - self.window_w)/2)
        coord_y = cell_size * self.current_position[0] - ((len(self.map) * cell_size - self.window_h)/2)
        pygame.draw.rect(self.window, (150,150,150), (coord_x, coord_y, cell_size, cell_size), 0)

    
    def update(self, map, state, current_position, charge, orientacion):
        #Check all events
        for e in EVENTS.get():
            if e.type == LOCALS.QUIT:
                self.quit()
            if e.type == pygame.MOUSEBUTTONDOWN:
                cell_size = 15
                coord_x, coord_y = e.pos
                j = (coord_x + ((len(self.map[0]) * cell_size - self.window_w)//2))//cell_size
                i = (coord_y + ((len(self.map) * cell_size - self.window_h)//2))//cell_size
                map[i][j] = 3
                
        
        self.map = map
        self.state = self.map_state(state)
        self.current_position = current_position
        self.charge = charge
        self.orientacion = self.map_orientacion(orientacion)
        
        self.window.fill((255,255,255))
        state_text = self.font.render("State: " + self.state, False, (0,0,0))
        self.window.blit(state_text, (20,20))
        state_text = self.font.render("Battery: " + str(self.charge) + "/" + self.max_charge, False, (0,0,0))
        self.window.blit(state_text, (20,60))
        state_text = self.font.render("Orientation: " + str(self.orientacion), False, (0,0,0))
        self.window.blit(state_text, (20,100))
        self.paint_map()
        pygame.display.update()

# --------------------------------------------------------------------------

if __name__ == '__main__':
    Information()
