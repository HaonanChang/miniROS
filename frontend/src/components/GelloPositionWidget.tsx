import { JSX } from "react";
import { useSelector } from "react-redux";
import * as UUID from "uuid";
import { RootState } from "../redux/Store";
function spawnGelloBox(
	value: number,
	isAligned: boolean,
	isAlignState: boolean = true,
) {
	const bgColor =
		isAligned || !isAlignState ? "bg-current/10" : "bg-red-500/20";
	const outlineStyle =
		isAligned || !isAlignState ? "" : "border-red-500 border-2";

	const uid = UUID.v4().toString();

	return (
		<div
			key={uid}
			className={`flex justify-center items-center ${bgColor} p-2 w-10 h-5 transition-all rounded-md ${outlineStyle}`}
		>
			<span className="text-sm transition-all">{value.toFixed(2)}</span>
		</div>
	);
}

export default function GelloPositionWidget() {
	const gelloState = useSelector(
		(state: RootState) => state.robotInfo.gelloState,
	);

	const isEmpty = Object.keys(gelloState).length === 0;

	let leftBoxes: JSX.Element[] = [];
	let rightBoxes: JSX.Element[] = [];

	if (!isEmpty) {
		leftBoxes = gelloState["left"]?.deltas.map((value, index) =>
			spawnGelloBox(value, gelloState["left"]?.aligned[index]),
		);
		rightBoxes = gelloState["right"]?.deltas.map((value, index) =>
			spawnGelloBox(value, gelloState["right"]?.aligned[index]),
		);
	}

	return (
		<>
			{isEmpty ? null : (
				<div className="flex flex-col gap-y-2">
					<div className="flex flex-row gap-x-2">
						<p className="text-md font-bold flex-1">Left</p>
						<div className="flex flex-row gap-x-2 flex-4">
							{leftBoxes}
						</div>
					</div>

					<div className="flex flex-row gap-x-2">
						<p className="text-md font-bold flex-1">Right</p>
						<div className="flex flex-row gap-x-2 flex-4">
							{rightBoxes}
						</div>
					</div>
				</div>
			)}
		</>
	);
}
