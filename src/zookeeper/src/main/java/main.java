import java.util.concurrent.TimeUnit;

import org.apache.curator.RetryPolicy;
import org.apache.curator.framework.CuratorFramework;
import org.apache.curator.framework.CuratorFrameworkFactory;
import org.apache.curator.framework.recipes.locks.InterProcessMutex;
import org.apache.curator.retry.ExponentialBackoffRetry;

public class main {
	private static int maxWait = 9999;

	@SuppressWarnings("null")
	public static void main(String args[]) {

		RetryPolicy retryPolicy = new ExponentialBackoffRetry(1000, 3);

		String hostInput = null;
		String node = null;

		InterProcessMutex lock = null;

		if (args.length > 0) {
			hostInput = args[0];
			node = args[1];
		}

		System.out.println(hostInput);

		String zookeeperConnectionString = hostInput != null ? hostInput : "localhost:2181";
		CuratorFramework client = CuratorFrameworkFactory.newClient(zookeeperConnectionString, retryPolicy);

		/*
		 * -------------------------------- Listen for connection changes
		 * --------------------------------
		 **/

		client.start();

		String lockPath = "/" + node;


		lock = new InterProcessMutex(client, lockPath);
		
		
		
		try {
			if (lock != null && locks_aquired(lock)) {
				System.out.println("lock_accepted");
				// waiting for input before releasing lock
				int i = System.in.read();
				System.out.println(i);
				
				lock.release();
				
				lock = null;
			}

		} catch (Exception e) {
			System.out.println("Could not acquire lock");
			System.out.println(e.getMessage());
		}
		//In case you move around the truck in, the truck could be waiting for a continue signal
		//That's why we send one here. It won't effect the intersection management. 
		System.out.println("lock_accepted"); 
		System.exit(0);

	}

	public static boolean locks_aquired(InterProcessMutex lock) {
		boolean lock_acquired = true;

		try {
			if (!lock.acquire(maxWait, TimeUnit.SECONDS)) {
				lock_acquired = false;
			}
		} catch (Exception e) {
			System.out.println("Could not acquire lock");
			System.out.println(e.getMessage());
		}

		return lock_acquired;

	}
}
