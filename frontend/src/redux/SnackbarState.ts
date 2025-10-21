import { createSlice, PayloadAction } from "@reduxjs/toolkit";
import Logger from "../util/Logger";

export const DEFAULT_DURATION_MS = 1500;

export enum SnackbarType {
	INFO = "INFO",
	GOOD = "GOOD",
	WARNING = "WARNING",
	ERROR = "ERROR",
	CLOSED = "CLOSED",
}

export interface SnackbarState {
	type: SnackbarType;
	message: string;
	durationMs: number | undefined;
}

const initialState: SnackbarState = {
	type: SnackbarType.CLOSED,
	message: "",
	durationMs: 0,
};

const rdcSnackbarSlice = createSlice({
	name: "snackbar",
	initialState,
	reducers: {
		setSnackBar: (state, action: PayloadAction<SnackbarState>) => {
			const payload = action.payload;

			if (payload.durationMs === undefined) {
				payload.durationMs = DEFAULT_DURATION_MS;
			}

			// Do not open a snackbar if the message is empty
			if (payload.message === undefined || payload.message.length === 0) {
				Logger.warning("SnackbarState", "Cannot show empty message!");
				payload.type = SnackbarType.CLOSED;
			}

			Logger.debug("Snackbar fired!", JSON.stringify(payload));

			state.type = payload.type;
			state.message = payload.message;
			state.durationMs = payload.durationMs;
		},
		clearSnackbar: (state) => {
			state.type = SnackbarType.CLOSED;
			state.message = "";
			state.durationMs = 0;
		},
	},
});

export const { setSnackBar, clearSnackbar } = rdcSnackbarSlice.actions;

export default rdcSnackbarSlice.reducer;
