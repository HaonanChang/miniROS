import { createSlice, PayloadAction } from "@reduxjs/toolkit";

export interface ActionLockState {
	actionLock: boolean;
}

const initialState: ActionLockState = {
	actionLock: false,
};

const actionLockSlice = createSlice({
	name: "actionLock",
	initialState,
	reducers: {
		setActionLock: (state, action: PayloadAction<boolean>) => {
			state.actionLock = action.payload;
		},
	},
});

export const { setActionLock } = actionLockSlice.actions;

export default actionLockSlice.reducer;
