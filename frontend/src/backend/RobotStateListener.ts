import { Dispatch } from "@reduxjs/toolkit";
import Globals from "../misc/Globals";
import Logger from "../misc/Logger";
import {
	CommanderState,
	resolveRobotState,
	setCommanderState,
} from "../redux/RobotInfo";

export default class RobotStateListener {
	private static socket: WebSocket;

	private static has_started = false;
	private static dispatcher: Dispatch;

	public static async registerWsListeners() {
		await new Promise((resolve) => setTimeout(resolve, 200));

		const socket = new WebSocket(`ws://${Globals.getRobotAddr()}/states`);
		this.socket = socket;

		socket.onopen = () => {
			Logger.info("WebSocket connection established");
		};

		socket.onmessage = (event) => {
			//console.log("Message received:", event.data);

			// Update the robot state
			this.dispatcher(resolveRobotState(event.data));
		};

		socket.onerror = (error) => {
			console.error("WebSocket error:", error);
		};

		socket.onclose = async () => {
			this.dispatcher(setCommanderState(CommanderState.DEAD));

			await this.registerWsListeners();
			// this.dispatcher(setCommanderState(CommanderState.DEAD));

			// Logger.info("WebSocket connection closed");
		};
	}

	public static async start(dispatch: Dispatch) {
		this.dispatcher = dispatch;

		if (this.has_started) {
			return;
		}

		this.has_started = true;

		await this.registerWsListeners();
	}

	public static stop() {
		if (!this.has_started) {
			return;
		}

		this.has_started = false;

		if (this.socket) {
			this.socket.close();
		}
	}
}
