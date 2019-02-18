temp_sub = rossubscriber('/temperature');

while (true)
    temp_msg = receive(temp_sub);
    temp = temp_msg.Data;
    disp(['Temperature = ', num2str(temp)])
end