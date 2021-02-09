import MainLayout from "@/layouts/MainLayout.vue";
import ConsumptionInformation from "@/components/consumptions/ConsumptionInformation.vue";
import CycleInformation from "@/components/cycles/CycleInformation.vue";
import StationInformation from "@/components/station/StationInformation.vue";
import PlannedTrajectory from "@/components/trajectories/PlannedTrajectory.vue";
import RealTrajectory from "@/components/trajectories/RealTrajectory.vue";
import useVuetify from "@/hooks/useVuetify";

const wrapper = useVuetify(MainLayout);

describe("When mounting main layout", () => {
  it("Should mount", () => {
    expect(wrapper.vm).toBeTruthy();
  });

  it("Should contain consumption information", () => {
    expect(wrapper.findComponent(ConsumptionInformation).vm).toBeTruthy();
  });

  it("Should contain cycle information", () => {
    expect(wrapper.findComponent(CycleInformation).vm).toBeTruthy();
  });

  it("Should contain station information", () => {
    expect(wrapper.findComponent(StationInformation).vm).toBeTruthy();
  });

  it("Should contain planned trajectory", () => {
    expect(wrapper.findComponent(PlannedTrajectory).vm).toBeTruthy();
  });

  it("Should contain real trajectory", () => {
    expect(wrapper.findComponent(RealTrajectory).vm).toBeTruthy();
  });
});
