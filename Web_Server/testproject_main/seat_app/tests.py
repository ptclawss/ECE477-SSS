from django.test import TestCase

# Create your tests here.
# seat_app/tests.py
from .models import Seat

class SeatModelTests(TestCase):
    def setUp(self):
        # Set up data for the tests
        Seat.objects.create(row=1, seat_id='A1', occupancy=True)
        Seat.objects.create(row=2, seat_id='B1', occupancy=False)

    def test_seat_occupancy(self):
        """Seats with occupancy set to True should be occupied."""
        seat_1 = Seat.objects.get(seat_id='A1')
        seat_2 = Seat.objects.get(seat_id='B1')
        self.assertTrue(seat_1.occupancy)
        self.assertFalse(seat_2.occupancy)

    def test_seat_str_method(self):
        """The __str__ method of the Seat model should return a string."""
        seat = Seat.objects.get(seat_id='A1')
        self.assertEqual(str(seat), 'Row 1, Seat A1')

    def test_seat_creation(self):
        """Test creating new seats."""
        Seat.objects.create(row=3, seat_id='C1', occupancy=True)
        Seat.objects.create(row=3, seat_id='C2', occupancy=False)
        self.assertEqual(Seat.objects.filter(row=3).count(), 2)

    def test_seat_unique_constraint(self):
        """Test that seat_id is unique within a row."""
        with self.assertRaises(Exception):
            Seat.objects.create(row=1, seat_id='A1', occupancy=False)

    def test_seat_update(self):
        """Test updating seat occupancy."""
        seat = Seat.objects.get(seat_id='A1')
        seat.occupancy = False
        seat.save()
        updated_seat = Seat.objects.get(seat_id='A1')
        self.assertFalse(updated_seat.occupancy)

    def test_seat_deletion(self):
        """Test deleting seats."""
        seat = Seat.objects.get(seat_id='A1')
        seat.delete()
        self.assertEqual(Seat.objects.filter(seat_id='A1').count(), 0)
