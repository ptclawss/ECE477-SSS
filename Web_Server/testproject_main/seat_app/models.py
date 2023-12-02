from django.db import models

# Create your models here.
from django.db import models

class Seat(models.Model):
    row = models.IntegerField()
    seat_id = models.CharField(max_length=10)
    occupancy = models.BooleanField()
