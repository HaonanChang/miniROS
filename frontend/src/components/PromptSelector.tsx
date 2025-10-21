import { ArrowUpward } from "@mui/icons-material";
import {
	Fab,
	FormControl,
	InputLabel,
	MenuItem,
	OutlinedInput,
	Select,
} from "@mui/material";
import { useEffect, useState } from "react";
import { useDispatch, useSelector } from "react-redux";

import RobotRequest from "../backend/RobotRequest";
import { SnackbarType, setSnackBar } from "../redux/SnackbarState";
import { RootState } from "../redux/Store";
export default function PromptSelector() {
	const [prompt, setPrompt] = useState("");
	const [presets, setPresets] = useState<string[]>([]);
	const [presetSelected, setPresetSelected] = useState<string | null>(null);

	const isActionLocked = useSelector(
		(state: RootState) => state.actionLock.actionLock,
	);

	const activePrompt = useSelector(
		(state: RootState) => state.robotInfo.prompt,
	);
	const [isLoading, setIsLoading] = useState(false);
	const dispatch = useDispatch();

	async function fetchPresets() {
		setIsLoading(true);

		let res = undefined;

		// Keep fetching until we get it
		while (res === undefined) {
			res = await RobotRequest.getPromptOptions();

			if (res === undefined) {
				await new Promise((resolve) => setTimeout(resolve, 1000));
			}
		}

		setPresets(res as string[]);
		setIsLoading(false);
	}

	// Fetch the presets when the component mounts
	useEffect(() => {
		fetchPresets();
	}, []);

	const onUpdatePrompt = async () => {
		await RobotRequest.updatePrompt(prompt);
		dispatch(
			setSnackBar({
				message: `Prompt updated: ${prompt}`,
				type: SnackbarType.INFO,
				durationMs: 1000,
			}),
		);
		setPrompt("");
		setPresetSelected(null);
	};

	const uploadButton = () => {
		return (
			<div className="flex flex-row gap-2 items-center">
				<Fab
					size="small"
					color="primary"
					sx={{
						boxShadow: "none",
					}}
					onClick={onUpdatePrompt}
					disabled={isLoading || prompt === "" || !isActionLocked}
				>
					<ArrowUpward fontSize="small" />
				</Fab>
			</div>
		);
	};

	return (
		<div className="flex flex-col gap-y-2 min-w-[300px]">
			<p className="text-2xl font-bold text-start">Prompt</p>

			<div className="flex flex-row gap-2 items-center ">
				<OutlinedInput
					placeholder={activePrompt}
					className="flex-1"
					value={prompt}
					disabled={!isActionLocked}
					onChange={(e) => {
						setPrompt(e.target.value);
						setPresetSelected(null);
					}}
					endAdornment={uploadButton()}
				/>
			</div>

			<FormControl fullWidth variant="outlined" className="mt-2">
				<InputLabel id="prompt-preset-label">
					Or select from preset prompts
				</InputLabel>
				<Select
					labelId="prompt-preset-label"
					id="prompt-preset"
					value={presetSelected ?? ""}
					onChange={(e) => {
						setPresetSelected(e.target.value as string);
						setPrompt(e.target.value as string);
					}}
					label="Or select from preset prompts"
					disabled={isLoading || !isActionLocked}
				>
					{presets.map((presetPrompt) => (
						<MenuItem key={presetPrompt} value={presetPrompt}>
							{presetPrompt}
						</MenuItem>
					))}
				</Select>
			</FormControl>

			{/* <div className="flex flex-row gap-2 justify-end">
				<Button
					variant="contained"
					color="primary"
					className="mt-2"
					disabled={isLoading || prompt === ""}
					sx={{
						textTransform: "none",
					}}
					onClick={onUpdatePrompt}
				>
					<ArrowUpward />
					<p className="ml-2">Update</p>
				</Button>
			</div> */}
		</div>
	);
}
