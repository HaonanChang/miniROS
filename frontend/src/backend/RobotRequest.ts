import { Dispatch } from "@reduxjs/toolkit";
import { CommanderState, setIsDataSaving } from "../redux/RobotInfo";
import {
	setSnackBar,
	SnackbarState,
	SnackbarType,
} from "../redux/SnackbarState";
import { store } from "../redux/Store";
import Globals from "../util/Globals";
import Logger from "../util/Logger";

export default class RobotRequest {
	//@ts-ignore
	private static _dispatcher: Dispatch;

	public static async safeAction(
		actionFunction: () => Promise<void>,
	): Promise<void> {
		const isLocked = store.getState().actionLock.actionLock;

		if (isLocked) {
			return;
		}

		actionFunction();
	}

	public static setDispatcher(dispatcher: Dispatch) {
		RobotRequest._dispatcher = dispatcher;
	}

	public static async resetCounter() {
		const res = await fetch(`http://${Globals.getRobotAddr()}/counter`, {
			method: "DELETE",
		});
		const data = await res.json();
		return data["counter"];
	}

	public static async getCounter() {
		const res = await fetch(`http://${Globals.getRobotAddr()}/counter`);
		const data = await res.json();
		return data["counter"];
	}

	/**
	 * Update the prompt for the robot
	 * @param prompt The prompt to update to
	 * @returns Whether the request was successful
	 */
	public static async updatePrompt(prompt: string) {
		const res = await fetch(
			`http://${Globals.getRobotAddr()}/update_prompt`,
			{
				method: "POST",
				body: JSON.stringify({ prompt }),
				headers: {
					"Content-Type": "application/json",
				},
			},
		);

		if (!res.ok) {
			await RobotRequest.onIssue(await res.json());
			return false;
		}

		return true;
	}

	/**
	 * Get the prompt options from the robot
	 * @returns The prompt options
	 */
	public static async getPromptOptions(): Promise<string[] | undefined> {
		const res = await fetch(`http://${Globals.getRobotAddr()}/prompts`);

		if (!res.ok) {
			await RobotRequest.onIssue(await res.json());
			return [];
		}

		const data = (await res.json()) as Record<string, unknown>;

		if (data["content"] === undefined) {
			// await RobotRequest.onIssue({
			// 	error: "Empty response content for /prompts API	",
			// });
			return undefined;
		}

		return ((data["content"] as Record<string, unknown>)["options"] ??
			undefined) as string[] | undefined;
	}

	public static async onIssue(data: Record<string, unknown>) {
		// const data = await res.json();

		// Logger.error("RobotRequest: ", JSON.stringify(data));

		if (data["content"] === undefined) {
			return;
		}

		RobotRequest._dispatcher(
			setSnackBar(<SnackbarState>{
				message: "ERROR: " + data["content"],
				type: SnackbarType.ERROR,
			}),
		);
	}

	public static async makeRobotRequest(route: string) {
		const res = await fetch(`http://${Globals.getRobotAddr()}/${route}`);

		const data = await res.json();

		Logger.info("Response: ", JSON.stringify(data));

		if (!res.ok) {
			await RobotRequest.onIssue(data);
			return false;
		}

		return true;
	}

	/**
	 * @deprecated Frontend does not call this directly anymore
	 * The correct action to perform is determined by the commander
	 */
	public static async startRecord() {
		await RobotRequest.makeRobotRequest("start_record");
	}

	/**
	 * @deprecated Frontend does not call this directly anymore
	 * The correct action to perform is determined by the commander
	 */
	public static async startInference() {
		await RobotRequest.makeRobotRequest("start_inference");
	}

	/**
	 * @deprecated Frontend does not call this directly anymore
	 * The correct action to perform is determined by the commander
	 */
	public static async markFailure() {
		await RobotRequest.makeRobotRequest("mark_failure");
	}

	/**
	 * @deprecated Frontend does not call this directly anymore
	 * The correct action to perform is determined by the commander
	 */
	public static async startTeleop() {
		await RobotRequest.makeRobotRequest("start_teleop");
	}

	/**
	 * Whenever the left paddle or "s" is pressed, this function is called
	 */
	public static async leftPaddle() {
		const prevState = store.getState().robotInfo.commander;

		// Only execute if the request is successful
		if (await RobotRequest.makeRobotRequest("left_paddle")) {
			let msg = undefined;

			// Show a snackbar to notify what the user has done
			switch (prevState) {
				case CommanderState.ACTIVE:
					msg = "Entering Recording Mode!";
					break;
				case CommanderState.AP_ALIGN:
					msg = "Entering Teleop Mode!";
					break;
				default:
					msg = undefined;
					break;
			}

			if (msg !== undefined) {
				RobotRequest._dispatcher(
					setSnackBar(<SnackbarState>{
						message: msg,
						type: SnackbarType.GOOD,
						durationMs: 2000,
					}),
				);
			}
		}
	}

	/**
	 * Whenever the middle paddle or "d" is pressed, this function is called
	 */
	public static async middlePaddle() {
		const prevState = store.getState().robotInfo.commander;

		// Only execute if the request is successful
		if (await RobotRequest.makeRobotRequest("middle_paddle")) {
			// TODO: This is coupling hack. Clean this up afterwards
			if (prevState === CommanderState.AP_INFERENCE) {
				RobotRequest._dispatcher(
					setSnackBar(<SnackbarState>{
						message: "Inference Failure marked!",
						type: SnackbarType.WARNING,
						durationMs: 2000,
					}),
				);
			}
			else if(prevState == CommanderState.RECORD) {
			const startRecordTime =
				store.getState().robotInfo.startRecordTimePosix ??
				new Date().getTime();

			const elapsed = new Date().getTime() - startRecordTime;

			const elapsed_seconds_str = (elapsed / 1000).toFixed(2);

			RobotRequest._dispatcher(setIsDataSaving(true));
			RobotRequest._dispatcher(
				setSnackBar(<SnackbarState>{
					message: `Saving session! (Recording Length: ${elapsed_seconds_str} s)`,
					type: SnackbarType.GOOD,
					durationMs: 2000,
				}),
			);
			}
		}
	}

	/**
	 * Whenever the right paddle or "q" is pressed, this function is called
	 */
	public static async rightPaddle() {
		if (await RobotRequest.makeRobotRequest("right_paddle")) {
			
			RobotRequest._dispatcher(
				setSnackBar(<SnackbarState>{
					message: `Emergency Stop paddle pressed!`,
					type: SnackbarType.INFO,
					durationMs: 2000,
				}),
			);
		}
	}

	public static async saveSession() {
		const res = await fetch(
			`http://${Globals.getRobotAddr()}/save_session`,
		);

		const data = await res.json();

		Logger.info("Response: ", JSON.stringify(data));

		if (!res.ok) {
			await RobotRequest.onIssue(data);
		} else {
			RobotRequest._dispatcher(setIsDataSaving(true));
			RobotRequest._dispatcher(
				setSnackBar(<SnackbarState>{
					message: "Saving session!",
					type: SnackbarType.GOOD,
					durationMs: 2000,
				}),
			);
		}
	}

	public static async discardSession() {
		const res = await fetch(
			`http://${Globals.getRobotAddr()}/discard_session`,
		);

		const data = await res.json();

		Logger.info("Response: ", JSON.stringify(data));

		if (!res.ok) {
			await RobotRequest.onIssue(data);
		} else {
			RobotRequest._dispatcher(setIsDataSaving(false));
			RobotRequest._dispatcher(
				setSnackBar(<SnackbarState>{
					message: "Discarding session!",
					type: SnackbarType.WARNING,
					durationMs: 2000,
				}),
			);
		}
	}
}
