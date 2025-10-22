import { createSlice, PayloadAction } from "@reduxjs/toolkit";
import Logger from "../misc/Logger";

export enum CommanderState {
	DEAD = "DEAD",
	WAITING = "WAITING",
	INIT = "INIT",
	ACTIVE = "ACTIVE",
	PREP_RECORD = "PREP_RECORD",
	RECORD = "RECORD",
	RESTORE = "RESTORE",
	STOPPED = "STOPPED",
	FAULT = "FAULT",

	AP_INFERENCE = "AP_INFERENCE",
	AP_ALIGN = "AP_ALIGN",
	// Hidden frontend states
	AP_ALIGN_READY = "AP_ALIGN_READY",
	AP_PRE_TELEOP = "AP_PRE_TELEOP",
	AP_TELEOP = "AP_TELEOP",
	AP_READY = "AP_READY",

	// For Some big robot
	EMERGENCY = "EMERGENCY",
	DRAG = "DRAG",
	REBOOT = "REBOOT",
}

export interface GelloState {
	joints: number[];
	aligned: boolean[];
	deltas: number[];
}

export interface RobotInfo {
	commander: CommanderState;
	robotData: Record<string, unknown>;
	gelloState: Record<string, GelloState>;
	isGelloHome: boolean;
	lastSaveFolderName: string;
	extra: string; // To be used for any extra information
	isDataSaving: boolean;
	isRecording: boolean;
	isAutopilot: boolean;
	counter: number;
	startRecordTimePosix: number | undefined;
	endRecordTimePosix: number | undefined;
	prompt: string;
}

const initialState: RobotInfo = {
	commander: CommanderState.DEAD,
	robotData: {},
	gelloState: {},
	isGelloHome: false,
	extra: "",
	lastSaveFolderName: "",
	isDataSaving: false,
	isRecording: false,
	isAutopilot: false,
	counter: 0,
	startRecordTimePosix: undefined,
	endRecordTimePosix: undefined,
	prompt: "",
};

const robotInfoSlice = createSlice({
	name: "robotInfo",
	initialState,
	reducers: {
		setIsDataSaving: (state, action: PayloadAction<boolean>) => {
			state.isDataSaving = action.payload;
		},

		resolveRobotState: (state, action: PayloadAction<string>) => {
			// eslint-disable-next-line @typescript-eslint/no-explicit-any
			const data = <Record<string, any>>JSON.parse(action.payload);

			try {
				const prevCommanderState = state.commander;

				state.commander = <CommanderState>(
					data["commander_state"].split(".")[1]
				);

				const isDifferentState = prevCommanderState !== state.commander;

				// Start Record
				// TODO: This is poorly designed, but it works for now
				if (
					isDifferentState &&
					new Set([
						CommanderState.RECORD,
						CommanderState.AP_INFERENCE,
					]).has(state.commander)
				) {
					state.startRecordTimePosix = new Date().getTime();
					state.isRecording = true;
					// Clear the data saving flag (Read and set right before RESTORE state, this is for UI)
					state.isDataSaving = false;
				}

				// End Record
				// TODO: This is poorly designed, but it works for now
				else if (
					isDifferentState &&
					// If the state is NOT one of the recording states, mark recording as stopped
					!new Set([
						CommanderState.RECORD,

						// For AutoPilot
						CommanderState.AP_TELEOP,
						CommanderState.AP_PRE_TELEOP,
						CommanderState.AP_ALIGN,
						CommanderState.AP_INFERENCE,
					]).has(state.commander)
				) {
					state.endRecordTimePosix = new Date().getTime();
					state.isRecording = false;
				}

				//Logger.info("Commander state", state.commander.toString());

				state.robotData = data["robot_data"];

				if (data["gello_state"] !== undefined) {
					const rawState = data["gello_state"] as Record<
						string,
						Record<string, unknown>
					>;

					for (const [hand, entry] of Object.entries(rawState)) {
						const joints = entry["joints"] as number[];
						const aligned = entry["aligned"] as boolean[];
						const deltas = entry["deltas"] as number[];
						state.gelloState[hand] = {
							joints,
							aligned,
							deltas,
						};
					}
				}

				if (
					data["robotData"] !== undefined &&
					data["robotData"]["last_save_folder_name"] !== undefined
				) {
					state.lastSaveFolderName =
						data["robotData"]["last_save_folder_name"];
				}

				// Deal with other fields
				if (data["is_autopilot"] !== undefined) {
					state.isAutopilot = data["is_autopilot"];
				}

				if (data["is_gello_home"] !== undefined) {
					state.isGelloHome = data["is_gello_home"];
				}

				// Counter
				if (data["counter"] !== undefined) {
					state.counter = data["counter"];
				}

				if (data["prompt"] !== undefined) {
					state.prompt = data["prompt"];
				}
			} catch (e: unknown) {
				Logger.error(e as Error);
			}
		},

		setCommanderState: (state, action: PayloadAction<CommanderState>) => {
			state.commander = action.payload;
		},
		setRobotExtra: (state, action: PayloadAction<string>) => {
			state.extra = action.payload;
		},
	},
});

export const {
	setCommanderState,
	setRobotExtra,
	resolveRobotState,
	setIsDataSaving,
} = robotInfoSlice.actions;
export default robotInfoSlice.reducer;
