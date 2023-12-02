// seat_status.js

// Function to create a seat element
function createSeat(seatId, occupancy) {
    const seat = document.createElement('div');
    seat.className = 'seat';
    seat.innerHTML = `${seatId}${occupancy !== undefined ? `<br>${occupancy ? '💺' : '🔴'}` : ''}`;
    return seat;
}

// Function to display seats based on JSON data
function displaySeats(data) {
    const seatsContainer = document.getElementById('seats-container');
    
    // Clear existing content
    seatsContainer.innerHTML = '';
    data.forEach(rowData => {
        const rowElement = document.createElement('div');
        rowElement.className = 'row';

        const rowLabel = document.createElement('div');
        rowLabel.className = 'row-label';
        rowLabel.textContent = `Row ${rowData.row}`;
        rowElement.appendChild(rowLabel);

        const seatMap = {};
        rowData.seats.forEach(seatData => {
            seatMap[seatData.seat_id] = seatData.occupancy;
        });

        for (let i = 1; i <= 12; i++) {
            const seatId = `${rowData.row}${i}`;
            const occupancy = seatMap[seatId];
            const seat = createSeat(seatId, occupancy);
            rowElement.appendChild(seat);
        }

        seatsContainer.appendChild(rowElement);
    });
}


// Function to fetch and update data asynchronously
async function updateData() {
    try {
        const response = await fetch('/static/seat_app/output.json');
        const jsonData = await response.json();
        displaySeats(jsonData);
    } catch (error) {
        console.error('Error loading data:', error);
    }
}

// Call updateData every 1 seconds (adjust the interval as needed)
setInterval(updateData, 1000);

// Display seats when the page loads
window.onload = function () {
    updateData(); // Initial data load
};
