#!/usr/bin/env python3

class Rng:
 def __init__(self, a, b, c, x):
  self.a = a
  self.b = b
  self.c = c
  self.x = x
def new(self):
 self.x += 1
 self.x &= 0xff
 self.a = (self.a ^ self.c ^ self.x)
 self.b = (self.b + self.a) & 0xff
 self.c = (self.c + (self.b >> 1) ^ self.a) & 0xff
 return self.c

r = Rng(0, 0, 0, 0)

def rndchoice_with_disparity(rng, threshold, correction):
 choice = (threshold - correction) > rng.new()
 if (choice):
  disparity = 255 - threshold
 else:
  disparity = 0 - threshold
 return (choice, disparity)

def test_disparity(rng, threshold):
 disp = 0
 min_correction = 0
 max_correction = 0
 num_tests = 10000
 sum = 0
 for i in range(num_tests):
  correction = disp >> 8
  min_correction = min(min_correction, correction)
  max_correction = max(max_correction, correction)
  choice, disparity = rndchoice_with_disparity(rng, threshold, disp >> 8)
  disp += disparity
  if (choice):
   sum += 1
 avg = sum / num_tests
 return (avg, disp, min_correction, max_correction)
