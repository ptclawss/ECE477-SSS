from django.urls import path
from .views import seat_status

urlpatterns = [
    path('seat_status/', seat_status, name='seat_status'),
    # Additional URL patterns for your app can be added here
]
