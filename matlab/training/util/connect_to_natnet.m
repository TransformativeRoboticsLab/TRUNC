function natnetclient = connect_to_natnet()
    
    % create an instance of the natnet client class
	fprintf( 'Creating natnet class object\n' )
	natnetclient = natnet;

	% connect the client to the server (multicast over local loopback) -
	% modify for your network
	fprintf( 'Connecting to the server\n' )
	natnetclient.HostIP = '127.0.0.1';
	natnetclient.ClientIP = '127.0.0.1';
	natnetclient.ConnectionType = 'Multicast';
	natnetclient.connect;
	if (natnetclient.IsConnected == 0)
		fprintf( 'Client failed to connect\n' )
		fprintf( '\tMake sure the host is connected to the network\n' )
		fprintf( '\tand that the host and client IP addresses are correct\n\n' ) 
		return
    end

    fprintf('Successfully connected to NatNet\n');

end