from django.contrib import admin
from .models import Seat

class SeatAdmin(admin.ModelAdmin):
    list_display = ('row', 'seat_id', 'occupancy')  # Use actual field names of your Seat model
    search_fields = ('seat_id', 'row')  # Use actual field names of your Seat model

# Register the model with the custom admin class
admin.site.register(Seat, SeatAdmin)
