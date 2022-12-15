import pygame
import sys

def init():
    pygame.init()
    win = pygame.display.set_mode((100, 100))

def getKey(keyName):
    ans = False
    for eve in pygame.event.get():
        pass
    keyInput = pygame.key.get_pressed()
    myKey = getattr(pygame, 'K_{}'.format(keyName))
    if keyInput [myKey]:
        ans = True
    pygame.display.update()
    return ans

def main():
    if getKey('LEFT'):
        print('Key Left is pressed')
    if getKey('RIGHT'):
        print('Key Right is pressed')
    if getKey('UP'):
        print('Key UP is pressed')
    if getKey('DOWN'):
        print('Key Down is pressed')
    if getKey('ESCAPE'):
        pygame.quit()
        sys.exit()

if __name__ == '__main__':
    init()
    while True:
        main()
