
# Source - https://stackoverflow.com/a/67034403
# Posted by Alexander Hunter, modified by community. See post 'Timeline' for change history
# Retrieved 2026-02-25, License - CC BY-SA 4.0

class Button:
    def __init__(self, x, y, text, centre=True):
        self.colour = (0, 0, 0)
        self.text = text
        self.text = pygame.font.SysFont("comicsansms", 30).render(text, 1, self.colour)

        if centre:
            self.x = int(x - self.text.get_width() // 2)
            self.y = int(y - self.text.get_height() // 2)
        else:
            self.x = x
            self.y = y

        self.width = int(self.text.get_width())
        self.height = int(self.text.get_height())

    # draw method which displays the text and border of the button
    def draw(self, win):
        pygame.draw.rect(win, (255, 255, 255), (self.x, self.y, self.width, self.height), 2)
        win.blit(self.text, (self.x, self.y))

    # method which returns true when the mouse is over the button and false when it isn't
    def is_over(self):
        pos = pygame.mouse.get_pos()

        if pos[0] > self.x and pos[0] < self.x + self.width:
            if pos[1] > self.y and pos[1] < self.y + self.height:
                return True

        return False

    # method for detecting whether or not the button has been pressed and returns true if it has
    @staticmethod
    def is_pressed():
        mouse = pygame.mouse.get_pressed()

        if mouse[0]:
            return True